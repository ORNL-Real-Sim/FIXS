@echo off
REM One-click: build + open the 'farside' junction signal-stop variant in CarMaker + IPGMovie.
REM   approach = head on approach edge (x=-7)  | farside = head across junction (x=+5) | nohead = no head
call "%~dp0run_junction_test.bat" farside
