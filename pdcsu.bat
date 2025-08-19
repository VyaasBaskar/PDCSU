@echo off
setlocal

:: Function to print in red
set "PS=powershell -NoProfile -ExecutionPolicy Bypass -Command"

%PS% "Write-Host 'Compiling...' -ForegroundColor Green"
clang++ -std=c++17 -Wall -Iinclude src\test.cc -o test.exe
if %errorlevel% neq 0 exit /b %errorlevel%
%PS% "Write-Host 'Compilation finished.' -ForegroundColor Green"

%PS% "Write-Host 'Running test.exe...' -ForegroundColor Green"
%PS% "Write-Host '' -ForegroundColor Green"
%PS% ^
  "$sw=[System.Diagnostics.Stopwatch]::StartNew();" ^
  "& .\\test.exe;" ^
  "$code=$LASTEXITCODE;" ^
  "$sw.Stop();" ^
  "Write-Host '';" ^
  "Write-Host ('Execution time: {0:N3} ms' -f $sw.Elapsed.TotalMilliseconds) -ForegroundColor Green;" ^
  "exit $code"

set "ecode=%ERRORLEVEL%"
%PS% "Write-Host '' -ForegroundColor Green"
%PS% "Write-Host 'Cleaning up...' -ForegroundColor Green"
if exist test.exe del /q test.exe
%PS% "Write-Host 'Done.' -ForegroundColor Green"

exit /b %ecode%
