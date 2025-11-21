@echo off
setlocal

set "PS=powershell -NoProfile -ExecutionPolicy Bypass -Command"

%PS% "Write-Host 'Compiling units tests...' -ForegroundColor Green"
clang++ -std=c++17 -Wall -Wextra -Iinclude src\units_test.cc -o units_test.exe
if %errorlevel% neq 0 exit /b %errorlevel%
%PS% "Write-Host 'Compilation finished.' -ForegroundColor Green"

%PS% "Write-Host 'Running units_test.exe...' -ForegroundColor Green"
%PS% "Write-Host '' -ForegroundColor Green"
%PS% ^
  "$sw=[System.Diagnostics.Stopwatch]::StartNew();" ^
  "& .\\units_test.exe;" ^
  "$code=$LASTEXITCODE;" ^
  "$sw.Stop();" ^
  "Write-Host '';" ^
  "Write-Host ('Execution time: {0:N3} ms' -f $sw.Elapsed.TotalMilliseconds) -ForegroundColor Green;" ^
  "exit $code"

set "ecode=%ERRORLEVEL%"
%PS% "Write-Host '' -ForegroundColor Green"
%PS% "Write-Host 'Cleaning up...' -ForegroundColor Green"
if exist units_test.exe del /q units_test.exe
%PS% "Write-Host 'Done.' -ForegroundColor Green"

exit /b %ecode%

