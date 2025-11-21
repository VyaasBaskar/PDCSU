@echo off
setlocal

set "PS=powershell -NoProfile -ExecutionPolicy Bypass -Command"

%PS% "Write-Host 'Making sure build artifacts directory exists...' -ForegroundColor Green"
if not exist build mkdir build

%PS% "Write-Host 'Building precompiled header (via build.bat)...' -ForegroundColor Green"
call build.bat
if %errorlevel% neq 0 exit /b %errorlevel%

%PS% "Write-Host 'Compiling units_test with pdcsu.pch...' -ForegroundColor Green"
clang++ -std=c++17 -Iinclude -include pdcsu.h -include-pch build/pdcsu.pch src/units_test.cc -o build/units_test.exe
if %errorlevel% neq 0 exit /b %errorlevel%

%PS% "Write-Host 'Running units_test.exe...' -ForegroundColor Green"
build\units_test.exe

set "ecode=%ERRORLEVEL%"
%PS% "Write-Host '' -ForegroundColor Green"
%PS% "Write-Host 'Cleaning compiled test binary...' -ForegroundColor Green"
if exist build\units_test.exe del /q build\units_test.exe

exit /b %ecode%

