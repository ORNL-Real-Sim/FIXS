"""Unit tests for the PR intent parser (.github/scripts/pr_intent.py).

The cases below are the real shapes this repo's PR bodies take -- the point of
the parser is that it separates them, so the tests are built from history:
PR #265 ("Closes #264", closed the issue), the #191 series ("Part of", did not),
PR #253 (a sibling-repo close), and the unfilled template that every PR starts
from.

Simulator-free; runs under `python -m pytest tests/Python/unit/ -v`.
"""

import os
import sys

import pytest

_SCRIPTS = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))),
    ".github", "scripts")
sys.path.insert(0, _SCRIPTS)

from pr_intent import IssueRef, lint, parse_intent  # noqa: E402

REPO = "ORNL-Real-Sim/FIXS"


def closes(body):
    return [r.slug for r in parse_intent(body, REPO).closes]


def parts(body):
    return [r.slug for r in parse_intent(body, REPO).parts]


class TestClosingKeywords:
    """Every keyword GitHub honours, since contributors already write them."""

    @pytest.mark.parametrize("word", [
        "Closes", "closes", "Close", "Closed",
        "Fixes", "fixes", "Fix", "Fixed",
        "Resolves", "resolves", "Resolve", "Resolved",
    ])
    def test_keyword_variants(self, word):
        assert closes(f"## Related Issues\n{word} #264\n") == [f"{REPO}#264"]

    def test_pr_265_body_shape(self):
        # The body that should have closed #264 and did not, because GitHub
        # declines to link a PR whose base is not the default branch.
        body = ("## Summary\nThe setup menus mark what the setup runs.\n\n"
                "## Related Issues\nCloses #264\n")
        assert closes(body) == [f"{REPO}#264"]

    def test_multiple_issues(self):
        body = "Closes #70\nCloses #223\nCloses #238\n"
        assert closes(body) == [f"{REPO}#70", f"{REPO}#223", f"{REPO}#238"]

    def test_duplicates_collapse(self):
        assert closes("Closes #264. Also closes #264.") == [f"{REPO}#264"]


class TestPartOf:
    """Incremental work must never close its umbrella issue."""

    def test_part_of_does_not_close(self):
        intent = parse_intent("Part of #191", REPO)
        assert intent.closes == []
        assert [r.slug for r in intent.parts] == [f"{REPO}#191"]
        assert intent.declared

    def test_part_of_wins_over_close_for_same_issue(self):
        # The narrower claim is the safe one: a wrong close is silent data
        # loss, a missed close is visible in the backlog.
        body = "Closes #191\nPart of #191\n"
        assert closes(body) == []
        assert parts(body) == [f"{REPO}#191"]

    def test_mixed_close_and_part(self):
        body = "Closes #258\nPart of #191\n"
        assert closes(body) == [f"{REPO}#258"]
        assert parts(body) == [f"{REPO}#191"]


class TestCrossRepo:
    """Sibling repos, which GitHub keywords cannot close under any branch."""

    def test_short_sibling_form(self):
        assert closes("Closes FIXS_Applications#21") == [
            "ORNL-Real-Sim/FIXS_Applications#21"]

    def test_fully_qualified_form(self):
        assert closes("Closes ORNL-Real-Sim/Digital-Twin-Library#4") == [
            "ORNL-Real-Sim/Digital-Twin-Library#4"]

    def test_is_local_to(self):
        assert IssueRef(REPO, 1).is_local_to(REPO)
        assert not IssueRef("ORNL-Real-Sim/FIXS_Applications", 1).is_local_to(REPO)

    def test_foreign_and_local_split(self):
        intent = parse_intent("Closes #258\nCloses FIXS_Applications#27\n", REPO)
        assert [r.slug for r in intent.local_closes(REPO)] == [f"{REPO}#258"]
        assert [r.slug for r in intent.foreign_closes(REPO)] == [
            "ORNL-Real-Sim/FIXS_Applications#27"]


class TestNoIssue:
    def test_no_issue_with_reason(self):
        intent = parse_intent("No issue: release line sync", REPO)
        assert intent.no_issue_reason == "release line sync"
        assert intent.declared

    def test_no_issue_dash_separator(self):
        assert parse_intent("No issue - merge plumbing", REPO).no_issue_reason \
            == "merge plumbing"

    def test_bare_no_issue_is_not_a_declaration(self):
        # Without a reason it is indistinguishable from prose.
        assert not parse_intent("No issue", REPO).declared


class TestNoiseRejection:
    """The parser must not read intent out of quoted or templated text."""

    def test_template_comment_is_stripped(self):
        # The PR template's own instructions contain literal examples of every
        # form; if they parsed, the empty template would satisfy the gate.
        body = ("<!--\nTITLE FORMAT: #<issue_number>/<short_description>\n"
                "  no issue: file one first\n-->\n\n## Summary\nStuff.\n")
        assert not parse_intent(body, REPO).declared

    def test_fenced_block_is_stripped(self):
        # This repo's PR bodies quote logs and prior bodies constantly. A
        # "Closes #264" inside a fence is evidence, not intent.
        body = "Quoting the old PR:\n\n```\nCloses #264\n```\n\nPart of #290\n"
        assert closes(body) == []
        assert parts(body) == [f"{REPO}#290"]

    def test_unfilled_placeholder_is_not_a_declaration(self):
        ok, msg = lint("## Related Issues\nCloses #___\n", REPO)
        assert not ok
        assert "placeholder" in msg

    def test_file_line_reference_is_not_an_issue(self):
        assert closes("see the note in run_cosim.py#L20 - fixes the drift") == []

    def test_prose_keyword_mid_sentence_is_not_a_declaration(self):
        # PR #228's real body: "the exact signal-side analogue of the vehicle
        # bug fixed in #176". A mid-sentence keyword is describing past work,
        # not declaring intent, and closing #176 off it would be silent data
        # loss. This is why declarations are line-anchored.
        body = ("`SignalSubscription` with `all` is a silent no-op - the exact "
                "signal-side analogue of the vehicle bug fixed in #176. This "
                "makes it functional.\n")
        assert closes(body) == []
        assert not parse_intent(body, REPO).declared

    def test_word_before_hash_is_not_a_repo_name(self):
        # The same body once resolved to the repo "in", because an optional
        # \s* let the preceding English word be read as the repo part.
        body = "Closes #176\nAlso mirrors the handling added in #176.\n"
        refs = parse_intent(body, REPO).closes
        assert [r.repo for r in refs] == [REPO]

    @pytest.mark.parametrize("line,expected", [
        ("Closes #264", [f"{REPO}#264"]),
        ("- Closes #264", [f"{REPO}#264"]),
        ("* **Closes** #264", [f"{REPO}#264"]),
        ("  Closes #264", [f"{REPO}#264"]),
        ("This closes #264 nicely", []),
        ("and it fixes #264", []),
    ])
    def test_line_anchoring(self, line, expected):
        assert closes(line + "\n") == expected

    def test_bare_title_reference_is_not_closing(self):
        # The whole reason this parser exists: "#258/..." means "closes" on
        # PR #269 and "one of twelve increments" across the #191 series, so a
        # bare reference is never a closing signal.
        assert not parse_intent(
            "#191/stamp v0.9.0-alpha rolling tag + cache yaml-cpp", REPO).declared


class TestLint:
    def test_empty_body_fails_with_guidance(self):
        ok, msg = lint("", REPO)
        assert not ok
        assert "Closes #N" in msg and "Part of #N" in msg

    def test_none_body_does_not_crash(self):
        ok, _ = lint(None, REPO)
        assert not ok

    @pytest.mark.parametrize("body,expected", [
        ("Closes #264", "closes"),
        ("Part of #191", "part of"),
        ("No issue: release sync", "no issue"),
    ])
    def test_each_declaration_passes(self, body, expected):
        ok, msg = lint(body, REPO)
        assert ok
        assert expected in msg
