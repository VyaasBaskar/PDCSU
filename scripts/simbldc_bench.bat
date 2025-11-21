@echo off
setlocal

set "PS=powershell -NoProfile -ExecutionPolicy Bypass -Command"

%PS% "Write-Host 'Compiling simbldc benchmark...' -ForegroundColor Green"
clang++ -std=c++17 -Wall -Wextra -Iinclude src\simbldc_bench.cc -o simbldc_bench.exe
if %errorlevel% neq 0 exit /b %errorlevel%
%PS% "Write-Host 'Compilation finished.' -ForegroundColor Green"

%PS% "Write-Host 'Running simbldc_bench.exe...' -ForegroundColor Green"
%PS% "Write-Host '' -ForegroundColor Green"
%PS% ^
  "$sw=[System.Diagnostics.Stopwatch]::StartNew();" ^
  "& .\\simbldc_bench.exe;" ^
  "$code=$LASTEXITCODE;" ^
  "$sw.Stop();" ^
  "Write-Host '';" ^
  "Write-Host ('Execution time: {0:N3} ms' -f $sw.Elapsed.TotalMilliseconds) -ForegroundColor Green;" ^
  "exit $code"

set "ecode=%ERRORLEVEL%"
%PS% "Write-Host '' -ForegroundColor Green"
%PS% "Write-Host 'Cleaning up...' -ForegroundColor Green"
if exist simbldc_bench.exe del /q simbldc_bench.exe
%PS% "Write-Host 'Done.' -ForegroundColor Green"

exit /b %ecode%

