param(
  [string]$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path,
  [string]$Remote = 'origin',
  [string]$Branch = 'main',
  [string]$BoardPath = 'hardware/esc/esc/esc.kicad_pcb',
  [int]$DebounceMilliseconds = 750
)

$ErrorActionPreference = 'Stop'
$git = 'git'
$boardFullPath = Join-Path $RepoRoot $BoardPath
$boardDir = Split-Path -Parent $boardFullPath
$boardName = Split-Path -Leaf $boardFullPath

function Get-BoardHash {
  param([string]$Path)
  if (-not (Test-Path -LiteralPath $Path)) { return $null }
  return (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash
}

Push-Location $RepoRoot
try {
  $prev = Get-BoardHash $boardFullPath
  $state = [hashtable]::Synchronized(@{ Pending = $false })
  $watcher = New-Object System.IO.FileSystemWatcher $boardDir, $boardName
  $watcher.NotifyFilter = [System.IO.NotifyFilters]'LastWrite,Size,FileName'
  $watcher.IncludeSubdirectories = $false
  $watcher.EnableRaisingEvents = $true

  Register-ObjectEvent -InputObject $watcher -EventName Changed -Action { $state.Pending = $true } | Out-Null
  Register-ObjectEvent -InputObject $watcher -EventName Created -Action { $state.Pending = $true } | Out-Null
  Register-ObjectEvent -InputObject $watcher -EventName Renamed -Action { $state.Pending = $true } | Out-Null

  Write-Host "Watching $BoardPath on $Branch. Press Ctrl+C to stop."

  while ($true) {
    Start-Sleep -Milliseconds $DebounceMilliseconds
    if (-not $state.Pending) { continue }
    $state.Pending = $false

    $cur = Get-BoardHash $boardFullPath
    if ($cur -eq $prev) { continue }
    $prev = $cur
    if (-not $cur) { continue }

    $branchName = (& $git branch --show-current).Trim()
    if ($branchName -ne $Branch) {
      Write-Host "Board changed, but current branch is '$branchName' not '$Branch'. Skipping auto-push."
      continue
    }

    & $git add -- $BoardPath
    if ($LASTEXITCODE -ne 0) { continue }

    & $git diff --cached --quiet -- $BoardPath
    if ($LASTEXITCODE -eq 0) { continue }

    $msg = "Auto-sync ESC PCB $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"
    $env:ALLOW_ESC_PCB_COMMIT = '1'
    & $git commit -m $msg
    if ($LASTEXITCODE -ne 0) { throw "Commit failed" }

    & $git push --force-with-lease $Remote "$Branch"
    if ($LASTEXITCODE -ne 0) { throw "Push failed" }

    Write-Host "Synced and force-pushed ESC PCB to $Remote/$Branch."
  }
}
finally {
  Get-EventSubscriber | Unregister-Event -ErrorAction SilentlyContinue
  Pop-Location
}
