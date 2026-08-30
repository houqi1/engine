$log = 'C:\Users\win11\engine\build\Debug\voxel_launch.log'
function Log([string]$msg) {
  $line = ('[{0}] {1}' -f (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'), $msg)
  Add-Content -Path $log -Value $line -Encoding UTF8
}

try {
  $root = 'C:\Users\win11\engine'
  $dir = Join-Path $root 'build\Debug'
  $exe = Join-Path $dir 'vulkan_engine_voxel.exe'
  $sdkBin = 'C:\VulkanSDK\1.4.357.0\Bin'

  Log "start dir=$dir exeExists=$(Test-Path $exe)"
  if (-not (Test-Path $exe)) { throw "Missing exe: $exe" }

  if (Test-Path $sdkBin) {
    $env:PATH = "$sdkBin;$env:PATH"
    $env:VULKAN_SDK = 'C:\VulkanSDK\1.4.357.0'
  }

  $p = Start-Process -FilePath $exe -WorkingDirectory $dir -PassThru
  Log "launched pid=$($p.Id)"
} catch {
  Log ("ERROR: " + $_.Exception.Message)
  throw
}
