@echo off
REM The build entry point. The real script lives at scripts\dispatch\dispatch.bat,
REM beside the numbered component scripts it calls; this forwards to it so that
REM `dispatch.bat`, run from the repository root, is a command that works.
REM
REM CLAUDE.md, README.md and doc\BUILD.md have all printed it that way since the
REM repo was imported, and it has never existed here -- so the first command a new
REM contributor runs answered with "not recognized as an internal or external
REM command". A shim rather than eight doc edits, because the short form is the one
REM in everyone's muscle memory and in every side channel a doc edit cannot reach.
REM FIXS#299.
REM
REM Arguments pass through and the child's exit code becomes this one's: a wrapper
REM that swallowed either would put the real script out of reach of CI.
call "%~dp0scripts\dispatch\dispatch.bat" %*
exit /b %ERRORLEVEL%
