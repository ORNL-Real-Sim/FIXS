<!--
TITLE FORMAT: #<issue_number>/<short_description>
  multiple issues:  #55 #56/migrate to libtraci libsumo
  sibling repo:     FIXS_Applications#13/--doctor, --version, --log
                    (write the repo name in full -- a bare #13 resolves to FIXS
                     issue 13, which is a different piece of work)
  no issue:         file one first; if the change genuinely has no tracker
                    (release line syncs, merge plumbing), leave the title plain
-->

## Summary
What this PR does and why. Use your own sub-headings -- narrative structure is
fine, and usually better than a checklist for anything non-trivial.

## Related Issues
<!--
REQUIRED -- the `PR intent` check fails without one of these three lines.

  Closes #___            this PR finishes the issue
  Part of #___           incremental work; the issue stays open
  No issue: <reason>     deliberate (release line sync, merge plumbing)

Say it here even though the title already carries the number. A bare `#N/`
title reference means "finishes it" on some PRs and "one of twelve increments"
on others, and nothing downstream can tell them apart. Because this PR targets
a dev branch, GitHub never creates the closing link itself -- the automation
reads this line instead, labels the issue `landed-on-dev` at merge, and closes
it when the code reaches `main`. (#290)

Sibling repos take the qualified form: `Closes FIXS_Applications#21`. GitHub
cannot close those from a keyword at all, so the automation handles them
separately.
-->
Closes #___

## Environment
Only the tools you actually tested against -- leave the rest blank:
- Python version:
- MATLAB/Simulink/dSPACE version:
- SUMO version:
- VISSIM version:
- IPG CarMaker version:

## Checklist
- [ ] Code compiles/runs as expected
- [ ] Tests pass locally
- [ ] Documentation is updated (if applicable)
- [ ] Issue linked above

## Additional Notes (optional)
Screenshots, logs, known issues, or design decisions.
