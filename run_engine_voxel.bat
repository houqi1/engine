@echo off
setlocal
set "ROOT=%~dp0"
set "EXE=%ROOT%build\Debug\vulkan_engine_voxel.exe"
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
echo Starting Vulkan Engine Voxel Demo...
echo EXE=%EXE%
echo DIR=%DIR%

rem Use cmd start so this window can close without killing the demo.
start "VulkanEngineVoxel" /D "%DIR%" "%EXE%"
if errorlevel 1 (
  echo Failed to start.
  pause
  exit /b 1
)

timeout /t 2 /nobreak >nul
tasklist /FI "IMAGENAME eq vulkan_engine_voxel.exe" | find /I "vulkan_engine_voxel.exe" >nul
if errorlevel 1 (
  echo Process exited immediately. Check for vulkan_engine_voxel_crash.log in:
  echo   %DIR%
  pause
  exit /b 1
)

echo Running. Close this console anytime; the demo stays open.
echo Prefer double-clicking this bat in Explorer if launched from an IDE/agent.
timeout /t 2 /nobreak >nul
endlocal
