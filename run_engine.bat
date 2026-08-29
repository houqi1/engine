@echo off
setlocal
set "ROOT=%~dp0"
set "EXE=%ROOT%build\Debug\vulkan_engine.exe"
set "DIR=%ROOT%build\Debug"

if not exist "%EXE%" (
  echo [ERROR] Not found: %EXE%
  echo Build the project first.
  pause
  exit /b 1
)

set "VULKAN_SDK=C:\VulkanSDK\1.4.357.0"
set "PATH=%VULKAN_SDK%\Bin;%PATH%"

cd /d "%DIR%"
echo Starting Vulkan Engine...
echo EXE=%EXE%
echo DIR=%DIR%
start "VulkanEngine" /D "%DIR%" "%EXE%"
if errorlevel 1 (
  echo Failed to start.
  pause
  exit /b 1
)

rem Keep a console for a moment so failures are visible.
timeout /t 2 /nobreak >nul
tasklist /FI "IMAGENAME eq vulkan_engine.exe" | find /I "vulkan_engine.exe" >nul
if errorlevel 1 (
  echo Process exited immediately. Check for vulkan_engine_crash.log in:
  echo   %DIR%
  pause
  exit /b 1
)

echo Running. You can close this console window.
timeout /t 3 /nobreak >nul
endlocal
