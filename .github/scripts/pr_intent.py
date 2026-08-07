#!/usr/bin/env python3
"""Parse a pull request's declared issue intent.

FIXS feature PRs target ``dev_v0.X.0``, never the default branch, so GitHub
never links them to an issue (the closing keyword is only honoured when the PR
base *is* the default branch). Every downstream step -- labelling on dev-merge,
closing on main-arrival, release notes -- therefore has to read the intent out
of the PR body itself.

That only works if the intent is unambiguous, and historically it was not.
Across 73 merged dev-targeted PRs, 24 carried a closing keyword and 34 carried
a bare ``#N/`` title reference, which meant "this finishes it" on #269/#258 and
"one of twelve increments" on the #191 series. Identical syntax, opposite
meaning, and nothing after the fact can tell them apart. So a bare reference is
never treated as closing -- the author states which it is, once, in the body:

    Closes #264                     this PR finishes it
    Part of #191                    incremental work, issue stays open
    No issue: release line sync     deliberate, nothing to track

All three accept a cross-repo form (``FIXS_Applications#21``, or the fully
qualified ``ORNL-Real-Sim/FIXS_Applications#21``). Note that GitHub cannot close
cross-repo issues from a keyword under any circumstances, so those are surfaced
for the lifecycle job to handle explicitly rather than silently dropped.

This module is pure: no network, no ``gh``, no environment. It is imported by
close_landed.py and covered by tests/Python/unit/test_pr_intent.py.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass, field
from typing import Optional

# The repo whose issues a bare "#N" refers to. Overridden by --repo so the
# parser stays usable from a fork or a test.
DEFAULT_REPO = "ORNL-Real-Sim/FIXS"

DEFAULT_OWNER = "ORNL-Real-Sim"

# Closing keywords GitHub itself honours; accepted here for continuity with
# what contributors already write.
_CLOSE_WORDS = r"clos(?:e|es|ed)|fix(?:es|ed)?|resolv(?:e|es|ed)"

# A reference is "#12", "Repo#12", or "owner/Repo#12". The repo part must abut
# the "#" exactly as GitHub's own syntax requires -- an intervening \s* let the
# preceding English word be read as a repo name, which is how PR #228's prose
# ("the vehicle bug fixed in #176") once resolved to the repo "in".
_REF = r"(?:(?P<repo>[A-Za-z0-9._-]+(?:/[A-Za-z0-9._-]+)?))?#(?P<num>\d+)"

# Declarations are line-anchored: the keyword must open a line, allowing only a
# list bullet or bold marker before it. GitHub scans anywhere in a body, but
# GitHub also previews the link in the UI and only acts on the default branch.
# This automation acts unattended, and a false close is silent data loss, so it
# demands the deliberate shape the PR template asks for:
#
#     Closes #264          declaration      -> parsed
#   - **Closes** #264      declaration      -> parsed
#     ...bug fixed in #176 prose            -> ignored
_LINE_START = r"^[ \t]*(?:[-*+][ \t]+)?(?:\*\*|__)?[ \t]*"

_CLOSE_RE = re.compile(
    rf"{_LINE_START}(?:{_CLOSE_WORDS})(?:\*\*|__)?[ \t]+{_REF}",
    re.IGNORECASE | re.MULTILINE)
_PART_RE = re.compile(
    rf"{_LINE_START}part[ \t]+of(?:\*\*|__)?[ \t]+{_REF}",
    re.IGNORECASE | re.MULTILINE)
_NO_ISSUE_RE = re.compile(r"^\s*no\s+issue\s*[:\-]\s*(?P<reason>.+?)\s*$",
                          re.IGNORECASE | re.MULTILINE)

# An unfilled template placeholder ("Closes #___") must not read as a
# declaration -- that is the default state of every PR, and accepting it would
# make the gate unenforceable.
_PLACEHOLDER_RE = re.compile(rf"{_LINE_START}(?:{_CLOSE_WORDS})[ \t]+#_+",
                             re.IGNORECASE | re.MULTILINE)

# HTML comments carry the template's own instructions, which contain literal
# examples of every form above. Strip them before parsing or the template
# itself satisfies the gate.
_COMMENT_RE = re.compile(r"<!--.*?-->", re.DOTALL)

# Fenced code blocks quote logs and prior PR bodies (this repo's issues do it
# constantly). A "Closes #264" inside a quoted block is evidence, not intent.
_FENCE_RE = re.compile(r"```.*?```", re.DOTALL)


@dataclass(frozen=True)
class IssueRef:
    """One referenced issue, normalized to ``owner/repo`` + number."""

    repo: str
    number: int

    @property
    def slug(self) -> str:
        return f"{self.repo}#{self.number}"

    def is_local_to(self, repo: str) -> bool:
        return self.repo.lower() == repo.lower()


@dataclass
class Intent:
    """What a PR body declares about the issues it touches."""

    closes: list[IssueRef] = field(default_factory=list)
    parts: list[IssueRef] = field(default_factory=list)
    no_issue_reason: Optional[str] = None

    @property
    def declared(self) -> bool:
        """True when the author stated an intent of any of the three kinds."""
        return bool(self.closes or self.parts or self.no_issue_reason)

    def local_closes(self, repo: str) -> list[IssueRef]:
        return [r for r in self.closes if r.is_local_to(repo)]

    def foreign_closes(self, repo: str) -> list[IssueRef]:
        return [r for r in self.closes if not r.is_local_to(repo)]

    def to_dict(self) -> dict:
        return {
            "closes": [r.slug for r in self.closes],
            "parts": [r.slug for r in self.parts],
            "no_issue_reason": self.no_issue_reason,
            "declared": self.declared,
        }


def _normalize_repo(raw: Optional[str], default_repo: str) -> str:
    """Expand a reference's repo part to ``owner/repo``.

    ``None`` -> the default repo (a bare ``#12``).
    ``FIXS_Applications`` -> ``ORNL-Real-Sim/FIXS_Applications``; the PR
    template asks for the short sibling form, so it has to resolve against the
    same owner rather than being rejected.
    """
    if not raw:
        return default_repo
    if "/" in raw:
        return raw
    owner = default_repo.split("/")[0] if "/" in default_repo else DEFAULT_OWNER
    return f"{owner}/{raw}"


def _collect(pattern: re.Pattern, text: str, default_repo: str) -> list[IssueRef]:
    seen: list[IssueRef] = []
    for m in pattern.finditer(text):
        ref = IssueRef(_normalize_repo(m.group("repo"), default_repo),
                       int(m.group("num")))
        if ref not in seen:
            seen.append(ref)
    return seen


def strip_noise(body: str) -> str:
    """Remove template comments and fenced blocks before parsing."""
    return _FENCE_RE.sub("", _COMMENT_RE.sub("", body or ""))


def parse_intent(body: str, repo: str = DEFAULT_REPO) -> Intent:
    """Parse a PR body into its declared intent."""
    text = strip_noise(body)
    closes = _collect(_CLOSE_RE, text, repo)
    parts = _collect(_PART_RE, text, repo)

    # "Part of #N" wins over a stray "Closes #N" for the same issue: the
    # narrower claim is the safe one, since a wrong close is silent data loss
    # while a missed close is visible in the backlog.
    part_nums = {r for r in parts}
    closes = [r for r in closes if r not in part_nums]

    m = _NO_ISSUE_RE.search(text)
    reason = m.group("reason") if m else None
    return Intent(closes=closes, parts=parts, no_issue_reason=reason)


def lint(body: str, repo: str = DEFAULT_REPO) -> tuple[bool, str]:
    """Check a PR body declares an intent. Returns (ok, message)."""
    intent = parse_intent(body, repo)
    if intent.declared:
        bits = []
        if intent.closes:
            bits.append("closes " + ", ".join(r.slug for r in intent.closes))
        if intent.parts:
            bits.append("part of " + ", ".join(r.slug for r in intent.parts))
        if intent.no_issue_reason:
            bits.append(f"no issue ({intent.no_issue_reason})")
        return True, "; ".join(bits)

    if _PLACEHOLDER_RE.search(strip_noise(body)):
        return False, (
            "The 'Closes #___' placeholder is still unfilled. Replace it with "
            "the issue number, or say 'Part of #N' / 'No issue: <reason>'.")
    return False, (
        "No issue intent declared. Add exactly one of these to the PR body:\n"
        "  Closes #N            - this PR finishes issue N\n"
        "  Part of #N           - incremental work; N stays open\n"
        "  No issue: <reason>   - deliberate (release sync, merge plumbing)\n"
        "Sibling repos take the qualified form, e.g. 'Closes "
        "FIXS_Applications#21'.\n"
        "This is required because the PR targets a dev branch, where GitHub "
        "never links the issue itself -- the lifecycle automation reads this "
        "line instead, and a bare '#N/' title reference is not enough (it "
        "means 'closes' on some PRs and 'partial' on others).")


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--body-file", required=True,
                    help="file holding the PR body (use - for stdin)")
    ap.add_argument("--repo", default=DEFAULT_REPO,
                    help="repo a bare '#N' resolves against")
    ap.add_argument("--json", action="store_true",
                    help="print the parsed intent instead of linting")
    args = ap.parse_args(argv)

    body = sys.stdin.read() if args.body_file == "-" \
        else open(args.body_file, encoding="utf-8").read()

    if args.json:
        print(json.dumps(parse_intent(body, args.repo).to_dict(), indent=2))
        return 0

    ok, message = lint(body, args.repo)
    if ok:
        print(f"Intent declared: {message}")
        return 0
    print(f"::error title=PR intent not declared::{message}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
