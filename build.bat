@echo off
setlocal

set "PS=powershell -NoProfile -ExecutionPolicy Bypass -Command"

if "%~1"=="" (
  echo "Usage: build.bat <release_zip> [release_tag]"
  exit /b 1
) else (
  set "release_zip=%~1"
  if not "%~2"=="" (
    set "release_tag=%~2"
  ) else (
    set "release_tag=%release_zip%"
  )
)

%PS% "Write-Host 'Creating build directory...' -ForegroundColor Green"
if not exist build mkdir build

%PS% "Write-Host 'Generating pdcsu.pch...' -ForegroundColor Green"
clang++ -std=c++17 -x c++-header -Iinclude include/pdcsu.h -o build/pdcsu.pch
if %errorlevel% neq 0 exit /b %errorlevel%

set "release_temp=build\%release_zip%_tmp"
set "zip_dest=build\%release_zip%.zip"
set "release_folder=%release_zip%"

if exist "%release_temp%" rd /s /q "%release_temp%"
mkdir "%release_temp%"
mkdir "%release_temp%\%release_folder%"

%PS% "Write-Host 'Copying include/ tree for release packaging...' -ForegroundColor Green"
robocopy include "%release_temp%\%release_folder%" /E /NFL /NDL /NJH /NJS /NC /NS /NP >nul
if %errorlevel% geq 8 exit /b %errorlevel%

copy /Y build\pdcsu.pch "%release_temp%\%release_folder%\pdcsu.pch" >nul

%PS% "Write-Host 'Creating archive %zip_dest% with release_folder=%release_folder%' -ForegroundColor Green"
%PS% "Compress-Archive -Path \"%release_temp%\%release_folder%\" -DestinationPath \"%zip_dest%\" -Force"
if %errorlevel% neq 0 (
  rd /s /q "%release_temp%"
  exit /b %errorlevel%
)

rd /s /q "%release_temp%"

%PS% "Write-Host 'Precompiled header ready at build/pdcsu.pch and release archive %zip_dest%' -ForegroundColor Green"
endlocal