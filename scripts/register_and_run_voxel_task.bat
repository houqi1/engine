@echo off
setlocal
set TASK=VE_LaunchVoxelDemo
set TR=powershell.exe -NoProfile -ExecutionPolicy Bypass -File C:\Users\win11\engine\scripts\launch_voxel_detached.ps1

schtasks /Delete /TN "%TASK%" /F >nul 2>&1
schtasks /Create /TN "%TASK%" /TR "%TR%" /SC ONCE /ST 23:55 /SD 2099/01/01 /F /RL LIMITED
if errorlevel 1 (
  echo Failed to create scheduled task.
  exit /b 1
)
schtasks /Run /TN "%TASK%"
echo Task run requested.
endlocal
