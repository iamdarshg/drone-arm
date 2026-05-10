param(
  [string]$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

$ErrorActionPreference = 'Stop'
Push-Location $RepoRoot
try {
  git config core.hooksPath .githooks
  Write-Host "Configured core.hooksPath=.githooks"
  Write-Host "Run: powershell -ExecutionPolicy Bypass -File tools\watch-esc-main.ps1"
}
finally {
  Pop-Location
}
