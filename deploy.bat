@echo off
setlocal EnableDelayedExpansion

set "PS=powershell -NoProfile -ExecutionPolicy Bypass -Command"

if "%~1"=="" (
  echo Usage: deploy.bat ^<release_tag^> [commit_message]
  exit /b 1
)

set "release_tag=%~1"
set "release_zip=pdcsu_%release_tag:_=%"
set "commit_msg=[PDCSU Auto-deploy] Unlabeled"
if not "%~2"=="" set "commit_msg=%~2"

for /f "usebackq delims=" %%b in (`git rev-parse --abbrev-ref HEAD`) do set "current_branch=%%b"
if "%current_branch%"=="HEAD" (
  %PS% "Write-Host 'Cannot deploy from a detached HEAD.' -ForegroundColor Red"
  exit /b 1
)

%PS% "Write-Host 'Building precompiled headers and release archive...' -ForegroundColor Green"
call build.bat %release_zip% %release_tag%
if %errorlevel% neq 0 exit /b %errorlevel%

%PS% "Write-Host 'Staging all tracked files...' -ForegroundColor Green"
git add -A
if %errorlevel% neq 0 (
  %PS% "Write-Host 'git add failed.' -ForegroundColor Red"
  exit /b %errorlevel%
)

git commit --allow-empty -m "%commit_msg%"
if %errorlevel% neq 0 (
  %PS% "Write-Host 'git commit failed.' -ForegroundColor Red"
  exit /b %errorlevel%
)

%PS% "Write-Host 'Pushing %current_branch%...' -ForegroundColor Green"
git push origin "%current_branch%"
if %errorlevel% neq 0 (
  %PS% "Write-Host 'git push failed.' -ForegroundColor Red"
  exit /b %errorlevel%
)

if not exist "build\%release_zip%.zip" (
  %PS% "Write-Host 'Release archive missing: build\%release_zip%.zip' -ForegroundColor Red"
  exit /b 1
)

where gh >nul 2>&1
if %errorlevel% neq 0 (
  %PS% "Write-Host 'GitHub CLI (gh) not found. Install it to publish releases.' -ForegroundColor Red"
  exit /b 1
)

  %PS% "Write-Host 'Publishing GitHub release %release_tag%...' -ForegroundColor Green"
gh release view "%release_tag%" >nul 2>&1
if %errorlevel% equ 0 (
  %PS% "Write-Host 'Release already exists; deleting old tag %release_tag%...' -ForegroundColor Yellow"
  gh release delete "%release_tag%" --yes
  if %errorlevel% neq 0 (
    %PS% "Write-Host 'Failed to delete existing release %release_tag%.' -ForegroundColor Red"
    exit /b %errorlevel%
  )
)

gh release create "%release_tag%" "build\%release_zip%.zip" --title "%release_tag%" --notes "Automated release for %release_tag%" --target "%current_branch%"
if %errorlevel% neq 0 (
  %PS% "Write-Host 'gh release create failed.' -ForegroundColor Red"
  exit /b %errorlevel%
)

%PS% "Write-Host 'Deploy complete. Release available at https://github.com/VyaasBaskar/PDCSU/releases/tag/%release_tag%' -ForegroundColor Green"

endlocal

