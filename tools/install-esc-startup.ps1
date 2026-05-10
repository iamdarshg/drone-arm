param(
  [string]$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

$ErrorActionPreference = 'Stop'
$startupDir = Join-Path $env:APPDATA 'Microsoft\Windows\Start Menu\Programs\Startup'
$script = Join-Path $RepoRoot 'tools\watch-esc-main.ps1'
$launcher = Join-Path $startupDir 'DroneArm-ESC-PCB-AutoSync.cmd'

$content = "@echo off`r`nstart `"`" /min powershell -NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File `"$script`"`r`n"
[System.IO.File]::WriteAllText($launcher, $content, [System.Text.Encoding]::ASCII)

Write-Host "Installed startup launcher: $launcher"
Write-Host "It will start the ESC auto-sync watcher when you log in."
