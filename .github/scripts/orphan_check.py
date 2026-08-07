#!/usr/bin/env python3
"""Find merged PRs whose commits never reached a shipping branch.

GitHub marks a PR "Merged" when its head lands on its *base*. It never checks
whether that base itself goes anywhere. So a PR can be merged, green, and
closed-looking while its code has never executed on any branch that ships.

The specimen this exists for is FIXS #266. PR #267 merged into
``bug/254_spectator_frame_lag`` at 12:51:15Z; PR #256 had merged that same
branch into ``dev_v0.9.0`` at 12:50:35Z -- forty seconds earlier. Commit
3f08d945 is reachable only from two dead branches. GitHub still shows #267 as
Merged, and the fix has never run anywhere.

close_landed.py cannot catch this. It is event-driven: it reacts to commits
arriving on ``main``, and a commit that never arrives produces no event. The
defect is defined by absence, so it needs a sweep rather than a trigger --
hence a schedule.

Other causes in the same class: a base branch deleted before the merge
propagated, a force-push over a merge commit, an abandoned PR stack.

Reachability is tested with the compare API rather than ``git merge-base``:
FIXS packs ~840 MiB of history (see bundle-guard.yml), and cloning it weekly to
answer a question the API answers directly is pure waste.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import subprocess
import sys
from typing import Optional

TRACKING_TITLE = "[MAINTENANCE] Merged PRs whose commits never reached a shipping branch"
TRACKING_MARKER = "<!-- orphan-check:tracking -->"


def gh(*args: str, check: bool = True) -> str:
    # encoding pinned: see the same note in close_landed.py -- PR titles here
    # carry em-dashes that cp1252 cannot decode.
    proc = subprocess.run(("gh",) + args, capture_output=True, text=True,
                          encoding="utf-8", errors="replace")
    if proc.returncode != 0:
        if check:
            raise RuntimeError(
                f"gh {' '.join(args)} failed: "
                f"{((proc.stderr or proc.stdout) or '').strip()}")
        return ""
    return proc.stdout or ""


def gh_json(*args: str, check: bool = True):
    out = gh(*args, check=check)
    if not out or not out.strip():
        return None
    return json.loads(out)


def summary(text: str) -> None:
    print(text)
    path = os.environ.get("GITHUB_STEP_SUMMARY")
    if path:
        with open(path, "a", encoding="utf-8") as fh:
            fh.write(text + "\n")


def shipping_branches(repo: str) -> list[str]:
    """``main`` plus every ``dev*`` branch -- the branches that reach users."""
    payload = gh_json("api", f"/repos/{repo}/branches?per_page=100") or []
    names = [b["name"] for b in payload]
    ordered = [n for n in names if n == "main"]
    ordered += sorted((n for n in names if n.startswith("dev")), reverse=True)
    return ordered


def is_reachable(repo: str, branch: str, sha: str) -> Optional[bool]:
    """Is ``sha`` an ancestor of ``branch``?

    compare/base...head reports ``head``'s position relative to ``base``. An
    ancestor is "behind" (or "identical" when it is the tip itself); anything
    else means the commit is not in that branch's history. ``None`` on an API
    error, so a transient failure is never reported as an orphan.
    """
    payload = gh_json("api", f"/repos/{repo}/compare/{branch}...{sha}",
                      check=False)
    if not payload:
        return None
    return payload.get("status") in ("behind", "identical")


def merged_prs(repo: str, since: Optional[dt.datetime], limit: int) -> list[dict]:
    payload = gh_json(
        "pr", "list", "--repo", repo, "--state", "merged",
        "--limit", str(limit),
        "--json", "number,title,mergedAt,mergeCommit,baseRefName,url") or []
    if since is None:
        return payload
    keep = []
    for pr in payload:
        stamp = pr.get("mergedAt")
        if not stamp:
            continue
        merged_at = dt.datetime.fromisoformat(stamp.replace("Z", "+00:00"))
        if merged_at >= since:
            keep.append(pr)
    return keep


def find_orphans(repo: str, prs: list[dict], branches: list[str]) -> list[dict]:
    orphans = []
    for pr in prs:
        merge_commit = pr.get("mergeCommit") or {}
        sha = merge_commit.get("oid")
        if not sha:
            # Nothing to test -- an old PR whose merge commit GitHub no longer
            # reports. Absence of data is not evidence of an orphan.
            continue
        unknown = False
        for branch in branches:
            reachable = is_reachable(repo, branch, sha)
            if reachable:
                break
            if reachable is None:
                unknown = True
        else:
            if unknown:
                print(f"::warning::PR #{pr['number']}: reachability "
                      f"indeterminate (API error); not reported.")
                continue
            orphans.append({**pr, "sha": sha})
    return orphans


def render(repo: str, orphans: list[dict], branches: list[str]) -> str:
    lines = [
        TRACKING_MARKER,
        "",
        "Merged pull requests whose merge commit is not reachable from any "
        "shipping branch (" + ", ".join(f"`{b}`" for b in branches) + ").",
        "",
        "GitHub shows each of these as **Merged**. Their code has never run on "
        "a branch that ships -- the usual cause is a merge into a feature "
        "branch that had already been merged upstream, or a base branch "
        "deleted before the merge propagated.",
        "",
        "| PR | merged | base | merge commit |",
        "|---|---|---|---|",
    ]
    for o in orphans:
        lines.append(
            f"| [#{o['number']}]({o['url']}) {o['title']} | "
            f"{o.get('mergedAt', '?')} | `{o['baseRefName']}` | "
            f"[`{o['sha'][:8]}`](https://github.com/{repo}/commit/{o['sha']}) |")
    lines += [
        "",
        "Each needs a decision: cherry-pick onto the active dev branch, or "
        "close as superseded.",
        "",
        "_Maintained automatically by `.github/workflows/orphan-check.yml`._",
    ]
    return "\n".join(lines)


def find_tracking_issue(repo: str) -> Optional[int]:
    payload = gh_json(
        "issue", "list", "--repo", repo, "--state", "open", "--limit", "100",
        "--json", "number,title") or []
    for issue in payload:
        if issue["title"] == TRACKING_TITLE:
            return issue["number"]
    return None


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--repo", default=os.environ.get("GITHUB_REPOSITORY",
                                                     "ORNL-Real-Sim/FIXS"))
    ap.add_argument("--days", type=int, default=180,
                    help="only examine PRs merged within this window "
                         "(0 = all history)")
    ap.add_argument("--limit", type=int, default=300,
                    help="max merged PRs to fetch")
    ap.add_argument("--no-issue", action="store_true",
                    help="report only; do not touch the tracking issue")
    args = ap.parse_args(argv)

    since = None
    if args.days > 0:
        since = dt.datetime.now(dt.timezone.utc) - dt.timedelta(days=args.days)

    branches = shipping_branches(args.repo)
    if "main" not in branches:
        print("::error title=No shipping branches::Could not list branches.")
        return 1

    prs = merged_prs(args.repo, since, args.limit)
    print(f"Checking {len(prs)} merged PRs against "
          f"{', '.join(branches)}")
    orphans = find_orphans(args.repo, prs, branches)

    existing = None if args.no_issue else find_tracking_issue(args.repo)

    if not orphans:
        summary(f"### Orphan check\n\nNo orphaned merges among {len(prs)} "
                f"merged PRs. All merge commits are reachable from a shipping "
                f"branch.")
        if existing:
            gh("issue", "comment", str(existing), "--repo", args.repo,
               "--body", "All previously listed merges are now reachable from "
                         "a shipping branch. Closing.", check=False)
            gh("issue", "close", str(existing), "--repo", args.repo,
               "--reason", "completed", check=False)
        return 0

    body = render(args.repo, orphans, branches)
    summary(f"### Orphan check\n\n**{len(orphans)} orphaned merge(s) found.**\n\n"
            + "\n".join(f"- #{o['number']} `{o['sha'][:8]}` "
                        f"(base `{o['baseRefName']}`)" for o in orphans))

    if args.no_issue:
        print(body)
        return 0

    if existing:
        gh("issue", "edit", str(existing), "--repo", args.repo,
           "--body", body, check=False)
        print(f"Updated tracking issue #{existing}")
    else:
        url = gh("issue", "create", "--repo", args.repo,
                 "--title", TRACKING_TITLE, "--body", body,
                 "--label", "maintenance", check=False).strip()
        print(f"Opened tracking issue {url}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
