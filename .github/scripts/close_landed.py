#!/usr/bin/env python3
"""Drive an issue through the dev -> main branch model.

Two subcommands, one per transition:

  label-dev --pr N        A PR merged into dev_v0.X.0. Its ``Closes`` issues
                          are fixed but NOT shipped, so they stay open and get
                          the `landed-on-dev` label. The label set is then, at
                          any moment, exactly the fixed-but-unreleased backlog
                          -- and the changelog for the next release sync.

  close-main --before A --after B
                          Commits arrived on main. Resolve them to the PRs that
                          carried them, read each PR's declared intent, close
                          the issues, drop the label.

Why the resolution goes commit -> PR through the API rather than parsing
"Merge pull request #N" out of subjects: main's history already contains
cherry-picks of dev commits under different SHAs (f24d0c50 vs 5bb44a70), and
nothing guarantees a release stays a true merge. ``/commits/{sha}/pulls``
answers "which PR carried this" for squash, rebase and merge alike.

Nothing here clones the repo. FIXS packs ~840 MiB of history (see the note in
bundle-guard.yml about fetch-depth), so ancestry and commit ranges are resolved
with the compare API instead of git. The workflow sparse-checks out only this
directory.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Iterable, Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pr_intent import IssueRef, parse_intent  # noqa: E402

# PR titles in this repo carry em-dashes and arrows. The runner is UTF-8, but a
# local run on a Windows console is cp1252 and would die on the summary rather
# than on anything that matters.
for _stream in (sys.stdout, sys.stderr):
    if hasattr(_stream, "reconfigure"):
        _stream.reconfigure(encoding="utf-8", errors="replace")

LANDED_LABEL = "landed-on-dev"
LANDED_LABEL_COLOR = "0e8a16"
LANDED_LABEL_DESC = "Fixed on a dev branch; closes when it reaches main"

# The compare API returns at most 250 commits per page. A release sync is tens
# of commits, but a first push or a long-delayed sync can exceed it, and
# silently dropping the tail would silently skip issues.
COMPARE_PAGE = 250
MAX_COMPARE_PAGES = 20

# Concurrency for the commit -> PR resolution. Kept modest: GitHub's secondary
# rate limiter reacts to burst concurrency, not just total request count.
API_WORKERS = 8

ZERO_SHA = "0" * 40


class GhError(RuntimeError):
    pass


# Set by --dry-run. Reads still go out; writes are printed instead of sent, so
# a backfill over historical commits can be audited before it touches anything.
DRY_RUN = False

_WRITE_VERBS = {"POST", "PATCH", "PUT", "DELETE"}


def _is_write(args: tuple) -> bool:
    if args[:1] == ("issue",) and args[1:2] and args[1] in (
            "close", "comment", "edit", "create"):
        return True
    return any(a in _WRITE_VERBS for a in args)


def gh_ok(*args: str) -> bool:
    """Run a gh write and report whether it succeeded.

    Success is the exit code, never stdout. `gh issue close` and `gh issue
    edit` print their confirmation to STDERR and leave stdout empty, so testing
    the returned text marks every successful close as a failure -- which also
    silently skips the label removal that follows it.
    """
    if DRY_RUN:
        print(f"  [dry-run] gh {' '.join(args)}")
        return True
    proc = subprocess.run(("gh",) + args, capture_output=True, text=True,
                          encoding="utf-8", errors="replace")
    if proc.returncode != 0:
        warn(f"gh {' '.join(args[:3])} failed: "
             f"{((proc.stderr or proc.stdout) or '').strip()}")
        return False
    return True


def gh(*args: str, check: bool = True) -> str:
    """Run the gh CLI and return stdout."""
    if DRY_RUN and _is_write(args):
        print(f"  [dry-run] gh {' '.join(args)}")
        return "dry-run"
    # encoding is pinned rather than left to the locale: GitHub bodies in this
    # repo are full of em-dashes and box-drawing characters, and the Windows
    # default (cp1252) raises UnicodeDecodeError on them, which surfaces as a
    # dead reader thread and a None stdout rather than as an error.
    proc = subprocess.run(("gh",) + args, capture_output=True, text=True,
                          encoding="utf-8", errors="replace")
    if proc.returncode != 0:
        message = ((proc.stderr or proc.stdout) or "").strip()
        if check:
            raise GhError(f"gh {' '.join(args)} failed: {message}")
        return ""
    return proc.stdout or ""


def gh_json(*args: str, check: bool = True):
    out = gh(*args, check=check)
    if not out or not out.strip() or out == "dry-run":
        return None
    return json.loads(out)


def notice(msg: str) -> None:
    print(f"::notice::{msg}")


def warn(msg: str) -> None:
    print(f"::warning::{msg}")


def summary(lines: Iterable[str]) -> None:
    """Append to the job summary so a run is auditable without reading logs."""
    path = os.environ.get("GITHUB_STEP_SUMMARY")
    text = "\n".join(lines) + "\n"
    print(text)
    if path:
        with open(path, "a", encoding="utf-8") as fh:
            fh.write(text)


# --------------------------------------------------------------------------
# GitHub reads
# --------------------------------------------------------------------------

def compare_commits(repo: str, before: str, after: str) -> list[str]:
    """SHAs introduced by ``before..after``, via the compare API (no clone)."""
    shas: list[str] = []
    for page in range(1, MAX_COMPARE_PAGES + 1):
        payload = gh_json(
            "api",
            f"/repos/{repo}/compare/{before}...{after}"
            f"?per_page={COMPARE_PAGE}&page={page}")
        if not payload:
            break
        batch = payload.get("commits") or []
        shas.extend(c["sha"] for c in batch)
        total = payload.get("total_commits", len(shas))
        if len(shas) >= total or len(batch) < COMPARE_PAGE:
            break
    else:
        warn(f"compare {before}...{after} exceeded "
             f"{COMPARE_PAGE * MAX_COMPARE_PAGES} commits; the tail was not "
             f"examined and issues it carried will stay open.")
    return shas


def prs_for_commit(repo: str, sha: str) -> list[int]:
    payload = gh_json("api", f"/repos/{repo}/commits/{sha}/pulls", check=False)
    return [p["number"] for p in payload] if payload else []


def prs_for_commits(repo: str, shas: list[str]) -> list[int]:
    """PR numbers carrying any of ``shas``, in first-seen order.

    One API call per commit, run on a small thread pool: the calls are pure
    I/O, and serially this is ~1 s per commit, which turns a backfill over a
    few hundred commits into minutes. Reads only, so concurrency is safe;
    ordering is restored from the input list rather than completion order so
    the run is reproducible.
    """
    results: dict[str, list[int]] = {}
    with ThreadPoolExecutor(max_workers=API_WORKERS) as pool:
        futures = {pool.submit(prs_for_commit, repo, sha): sha for sha in shas}
        for done, future in enumerate(as_completed(futures), 1):
            sha = futures[future]
            try:
                results[sha] = future.result()
            except Exception as exc:  # a single commit must not sink the run
                warn(f"could not resolve PRs for {sha[:8]}: {exc}")
                results[sha] = []
            if done % 50 == 0:
                print(f"  resolved {done}/{len(shas)} commits")

    ordered: list[int] = []
    for sha in shas:
        for number in results.get(sha, []):
            if number not in ordered:
                ordered.append(number)
    return ordered


def pr_details(repo: str, number: int) -> dict:
    return gh_json("api", f"/repos/{repo}/pulls/{number}") or {}


def issue_state(ref: IssueRef, number: int) -> Optional[str]:
    payload = gh_json("api", f"/repos/{ref.repo}/issues/{number}", check=False)
    if not payload:
        return None
    # The issues endpoint also serves PRs; never act on one.
    if "pull_request" in payload:
        return "pull_request"
    return payload.get("state")


# --------------------------------------------------------------------------
# GitHub writes
# --------------------------------------------------------------------------

def ensure_label(repo: str) -> None:
    existing = gh_json("api", f"/repos/{repo}/labels/{LANDED_LABEL}",
                       check=False)
    if existing:
        return
    gh("api", "-X", "POST", f"/repos/{repo}/labels",
       "-f", f"name={LANDED_LABEL}",
       "-f", f"color={LANDED_LABEL_COLOR}",
       "-f", f"description={LANDED_LABEL_DESC}", check=False)


def add_label(ref: IssueRef) -> bool:
    return gh_ok("issue", "edit", str(ref.number), "--repo", ref.repo,
                 "--add-label", LANDED_LABEL)


def remove_label(ref: IssueRef) -> None:
    # Absent label -> 404, which is the expected case for anything that landed
    # before the label existed. Not an error worth surfacing.
    if DRY_RUN:
        print(f"  [dry-run] remove {LANDED_LABEL} from {ref.slug}")
        return
    subprocess.run(
        ("gh", "api", "-X", "DELETE",
         f"/repos/{ref.repo}/issues/{ref.number}/labels/{LANDED_LABEL}"),
        capture_output=True, text=True, encoding="utf-8", errors="replace")


def comment(ref: IssueRef, body: str) -> bool:
    return gh_ok("issue", "comment", str(ref.number), "--repo", ref.repo,
                 "--body", body)


def close_issue(ref: IssueRef, body: str) -> bool:
    # Comment first: if the close then fails, the issue still carries the
    # explanation, which is recoverable. The reverse leaves a bare close.
    if not comment(ref, body):
        return False
    return gh_ok("issue", "close", str(ref.number), "--repo", ref.repo,
                 "--reason", "completed")


# --------------------------------------------------------------------------
# label-dev
# --------------------------------------------------------------------------

def cmd_label_dev(args) -> int:
    repo = args.repo
    pr = pr_details(repo, args.pr)
    if not pr:
        warn(f"PR #{args.pr} not found.")
        return 0

    intent = parse_intent(pr.get("body") or "", repo)
    if not intent.closes:
        notice(f"PR #{args.pr} declares no closing issue "
               f"({len(intent.parts)} 'Part of'); nothing to label.")
        return 0

    ensure_label(repo)
    base = pr.get("base", {}).get("ref", "?")
    sha = (pr.get("merge_commit_sha") or "")[:8]
    rows = []
    for ref in intent.closes:
        if not ref.is_local_to(repo):
            rows.append(f"- {ref.slug} - sibling repo, not labelled here")
            continue
        state = issue_state(ref, ref.number)
        if state != "open":
            rows.append(f"- {ref.slug} - skipped (state: {state})")
            continue
        body = (
            f"Fixed on `{base}` by #{args.pr} (`{sha}`).\n\n"
            f"Labelled `{LANDED_LABEL}` and left open on purpose: the fix has "
            f"not reached `main` yet. It closes automatically when the release "
            f"sync merges `{base}` into `main`.")
        ok = add_label(ref) and comment(ref, body)
        rows.append(f"- {ref.slug} - {'labelled' if ok else 'FAILED'}")

    summary([f"### `{LANDED_LABEL}` from #{args.pr}", ""] + rows)
    return 0


# --------------------------------------------------------------------------
# close-main
# --------------------------------------------------------------------------

def cmd_close_main(args) -> int:
    repo = args.repo
    before, after = args.before, args.after

    if not before or before == ZERO_SHA:
        # Branch creation, or a force-push GitHub could not bound. Comparing
        # against an empty tree would walk the entire history and close
        # everything ever referenced, so refuse and let a human pass a range.
        warn("push event carries no usable 'before' SHA (branch creation or "
             "force-push). Skipping; re-run this workflow manually with an "
             "explicit range if issues need closing.")
        return 0
    if before == after:
        notice("Empty push range; nothing to do.")
        return 0

    shas = compare_commits(repo, before, after)
    if not shas:
        notice(f"No commits in {before[:8]}..{after[:8]}.")
        return 0

    print(f"Resolving {len(shas)} commits to their pull requests...")
    pr_numbers = prs_for_commits(repo, shas)

    closed, skipped, foreign, silent = [], [], [], []
    for number in pr_numbers:
        pr = pr_details(repo, number)
        if not pr:
            continue
        intent = parse_intent(pr.get("body") or "", repo)
        title = (pr.get("title") or "").strip()

        if not intent.closes:
            # A PR that references an issue in its title but declares nothing
            # in the body is the historical ambiguity pr_intent.py exists to
            # end: it means "finishes it" on some PRs and "one of twelve
            # increments" on others. Report, never guess.
            if not intent.declared and title.lstrip().startswith("#"):
                silent.append(f"- #{number} {title}")
            continue

        for ref in intent.closes:
            if not ref.is_local_to(repo):
                # GitHub cannot close cross-repo issues from a keyword under
                # any circumstances. github.token is scoped to this repo, so
                # this only works once a PAT with issues:write on the sibling
                # is provided; until then, report it.
                if close_issue(ref, _closing_comment(repo, number, after)):
                    closed.append(f"- {ref.slug} (cross-repo) via #{number}")
                else:
                    foreign.append(f"- {ref.slug} via #{number} - no write "
                                   f"access; close by hand")
                continue

            state = issue_state(ref, ref.number)
            if state != "open":
                skipped.append(f"- {ref.slug} - already {state}")
                continue
            if close_issue(ref, _closing_comment(repo, number, after)):
                remove_label(ref)
                closed.append(f"- {ref.slug} via #{number}")
            else:
                skipped.append(f"- {ref.slug} - close FAILED")

    lines = [f"### Issues landed on `main` ({len(shas)} commits, "
             f"{len(pr_numbers)} PRs)", ""]
    lines += ["**Closed**", ""] + (closed or ["- none"])
    if skipped:
        lines += ["", "**Skipped**", ""] + skipped
    if foreign:
        lines += ["", "**Sibling-repo issues needing a manual close**", "",
                  "_Add a PAT secret with `issues:write` on the sibling repo "
                  "to automate these._", ""] + foreign
    if silent:
        lines += ["", "**Landed with an issue in the title but no declared "
                  "intent**", "",
                  "_Not closed - a bare `#N` reference is not a closing "
                  "signal. The PR gate prevents new ones._", ""] + silent
    summary(lines)
    return 0


def _closing_comment(repo: str, pr: int, sha: str) -> str:
    return (f"Landed on `main` in "
            f"https://github.com/{repo}/commit/{sha} via #{pr}.\n\n"
            f"Closed by the issue-lifecycle workflow: this PR targeted a dev "
            f"branch, where GitHub does not create the closing link, so the "
            f"close happens when the code actually reaches `main`.")


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--repo", default=os.environ.get("GITHUB_REPOSITORY",
                                                     "ORNL-Real-Sim/FIXS"))
    ap.add_argument("--dry-run", action="store_true",
                    help="resolve and report, but do not comment, label or "
                         "close anything")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_label = sub.add_parser("label-dev", help="label issues a dev PR fixed")
    p_label.add_argument("--pr", type=int, required=True)
    p_label.set_defaults(func=cmd_label_dev)

    p_close = sub.add_parser("close-main", help="close issues that reached main")
    p_close.add_argument("--before", required=True)
    p_close.add_argument("--after", required=True)
    p_close.set_defaults(func=cmd_close_main)

    args = ap.parse_args(argv)
    global DRY_RUN
    DRY_RUN = args.dry_run
    if DRY_RUN:
        print("DRY RUN - no issue will be commented on, labelled or closed.\n")
    try:
        return args.func(args)
    except GhError as exc:
        print(f"::error title=GitHub API call failed::{exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
