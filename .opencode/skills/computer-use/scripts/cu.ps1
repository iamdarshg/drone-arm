# Computer Use helper for opencode on Windows
# Usage: powershell -File cu.ps1 <command> [args]
# Commands:
#   shot -Out <png> [-Window <title|pid>]      capture screen (or specific window) to PNG
#   windows                                     list top-level windows (pid, title, rect)
#   activate -Title <t> | -Pid <n>             bring window to foreground
#   move -X <n> -Y <n>                          move cursor (hover)
#   click -X <n> -Y <n> [-Double] [-Right] [-Mid]
#   drag -X1 -Y1 -X2 -Y2 [-Steps n]            left-button drag
#   mdrag -X1 -Y1 -X2 -Y2                       middle-button drag (pan)
#   scroll -X <n> -Y <n> -Delta <n>             wheel scroll (negative = down)
#   type -Text "..."                            send text to focused window
#   key -Keys "ctrl+s" ["enter" ...]            send key combo(s), space-separated
#   uiafind -Name <n> [-Parent <title>]         find UIA element(s) by name -> x,y center
#   uiaclick -Name <n> [-Parent <title>]        click center of UIA element by name
param()

$ErrorActionPreference = 'Stop'

if (-not ('CU.Native' -as [type])) {
Add-Type -TypeDefinition @"
using System;
using System.Runtime.InteropServices;
namespace CU {
  public class Native {
    [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
    [DllImport("user32.dll")] public static extern bool SetCursorPos(int x, int y);
    [DllImport("user32.dll")] public static extern void mouse_event(uint f, uint dx, uint dy, uint d, UIntPtr e);
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
    [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int cmd);
    [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
    [DllImport("user32.dll")] public static extern bool IsIconic(IntPtr h);
    [StructLayout(LayoutKind.Sequential)] public struct RECT { public int L, T, R, B; }
  }
}
"@
}
Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
[CU.Native]::SetProcessDPIAware() | Out-Null

function Get-TargetWindow([string]$Title, [int]$Pid2) {
  $cands = @()
  foreach ($p in (Get-Process | Where-Object { $_.MainWindowHandle -ne 0 })) {
    $match = ($Pid2 -gt 0 -and $p.Id -eq $Pid2) -or ($Title -and $p.MainWindowTitle -like "*$Title*")
    if ($match) { $cands += $p }
  }
  if ($cands.Count -eq 0) { throw "window not found (title='$Title' pid=$Pid2)" }
  return $cands[0]
}

function Focus-Window($proc) {
  if ([CU.Native]::IsIconic($proc.MainWindowHandle)) { [CU.Native]::ShowWindow($proc.MainWindowHandle, 9) | Out-Null; Start-Sleep -Milliseconds 300 }
  [CU.Native]::SetForegroundWindow($proc.MainWindowHandle) | Out-Null
  Start-Sleep -Milliseconds 400
}

function Convert-SendKeys([string]$combo) {
  # "ctrl+s" -> "^s"; "alt+f4" -> "%{F4}"; "shift+tab" -> "+{TAB}"; "enter" -> "{ENTER}"
  $map = @{ enter='{ENTER}'; tab='{TAB}'; esc='{ESC}'; escape='{ESC}'; space=' '; backspace='{BACKSPACE}';
           delete='{DELETE}'; del='{DELETE}'; insert='{INSERT}'; home='{HOME}'; end='{END}';
           pgup='{PGUP}'; pgdn='{PGDN}'; left='{LEFT}'; right='{RIGHT}'; up='{UP}'; down='{DOWN}';
           f1='{F1}';f2='{F2}';f3='{F3}';f4='{F4}';f5='{F5}';f6='{F6}';f7='{F7}';f8='{F8}';
           f9='{F9}';f10='{F10}';f11='{F11}';f12='{F12}' }
  $parts = $combo.ToLower() -split '\+'
  $prefix = ''; $last = $parts[-1]
  foreach ($p in $parts[0..($parts.Count-2)]) {
    switch ($p) { 'ctrl' {$prefix+='^'} 'alt' {$prefix+='%'} 'shift' {$prefix+='+'} default {throw "unknown mod '$p'"} }
  }
  if ($map.ContainsKey($last)) { return $prefix + $map[$last] }
  if ($last.Length -eq 1) { return $prefix + $last }
  throw "unknown key '$last'"
}

$cmd = $args[0]
$rest = @(); if ($args.Count -gt 1) { $rest = $args[1..($args.Count-1)] }
$p = @{}; for ($i=0; $i -lt $rest.Count; $i+=2) { $p[$rest[$i].TrimStart('-').ToLower()] = $rest[$i+1] }

switch ($cmd) {
  'shot' {
    $out = if ($p['out']) { $p['out'] } else { Join-Path $env:TEMP ("cu_" + (Get-Date -Format 'HHmmssfff') + ".png") }
    $pidArg = 0; if ($p.ContainsKey('pid')) { $pidArg = [int]$p['pid'] }
    if ($p['window']) {
      $proc = Get-TargetWindow $p['window'] $pidArg
      Focus-Window $proc
      $r = New-Object CU.Native+RECT
      [CU.Native]::GetWindowRect($proc.MainWindowHandle, [ref]$r) | Out-Null
      $w = $r.R - $r.L; $h = $r.B - $r.T
      $bmp = New-Object System.Drawing.Bitmap($w, $h)
      $g = [System.Drawing.Graphics]::FromImage($bmp)
      $g.CopyFromScreen($r.L, $r.T, 0, 0, (New-Object System.Drawing.Size($w, $h)))
    } else {
      $vs = [System.Windows.Forms.SystemInformation]::VirtualScreen
      $bmp = New-Object System.Drawing.Bitmap($vs.Width, $vs.Height)
      $g = [System.Drawing.Graphics]::FromImage($bmp)
      $g.CopyFromScreen($vs.X, $vs.Y, 0, 0, (New-Object System.Drawing.Size($vs.Width, $vs.Height)))
    }
    $g.Dispose(); $bmp.Save($out, [System.Drawing.Imaging.ImageFormat]::Png); $bmp.Dispose()
    Write-Output $out
  }
  'windows' {
    foreach ($pr in (Get-Process | Where-Object { $_.MainWindowHandle -ne 0 })) {
      $r = New-Object CU.Native+RECT
      [CU.Native]::GetWindowRect($pr.MainWindowHandle, [ref]$r) | Out-Null
      "{0}`t{1}`t({2},{3})-({4},{5})" -f $pr.Id, $pr.MainWindowTitle, $r.L, $r.T, $r.R, $r.B
    }
  }
  'activate' {
    $pidArg = 0; if ($p.ContainsKey('pid')) { $pidArg = [int]$p['pid'] }
    $proc = Get-TargetWindow $p['title'] $pidArg; Focus-Window $proc; Write-Output "activated $($proc.Id): $($proc.MainWindowTitle)"
  }
  'move'    { [CU.Native]::SetCursorPos([int]$p['x'], [int]$p['y']) | Out-Null; Start-Sleep -Milliseconds 120 }
  'click'   {
    [CU.Native]::SetCursorPos([int]$p['x'], [int]$p['y']) | Out-Null; Start-Sleep -Milliseconds 150
    $down=0x0002; $up=0x0004
    if ($p['right']) { $down=0x0008; $up=0x0010 } elseif ($p['mid']) { $down=0x0020; $up=0x0040 }
    [CU.Native]::mouse_event($down,0,0,0,[UIntPtr]::Zero); Start-Sleep -Milliseconds 60
    [CU.Native]::mouse_event($up,0,0,0,[UIntPtr]::Zero)
    if ($p['double']) { Start-Sleep -Milliseconds 90; [CU.Native]::mouse_event($down,0,0,0,[UIntPtr]::Zero); Start-Sleep -Milliseconds 60; [CU.Native]::mouse_event($up,0,0,0,[UIntPtr]::Zero) }
    Start-Sleep -Milliseconds 120
  }
  'drag'    {
    $steps = if ($p['steps']) { [int]$p['steps'] } else { 25 }
    [CU.Native]::SetCursorPos([int]$p['x1'], [int]$p['y1']) | Out-Null; Start-Sleep -Milliseconds 200
    [CU.Native]::mouse_event(0x0002,0,0,0,[UIntPtr]::Zero); Start-Sleep -Milliseconds 150
    for ($i=1; $i -le $steps; $i++) {
      $x = [int]([int]$p['x1'] + ([int]$p['x2']-[int]$p['x1']) * $i / $steps)
      $y = [int]([int]$p['y1'] + ([int]$p['y2']-[int]$p['y1']) * $i / $steps)
      [CU.Native]::SetCursorPos($x, $y) | Out-Null; Start-Sleep -Milliseconds 20
    }
    Start-Sleep -Milliseconds 120
    [CU.Native]::mouse_event(0x0004,0,0,0,[UIntPtr]::Zero)
  }
  'mdrag'   {
    [CU.Native]::SetCursorPos([int]$p['x1'], [int]$p['y1']) | Out-Null; Start-Sleep -Milliseconds 150
    [CU.Native]::mouse_event(0x0020,0,0,0,[UIntPtr]::Zero); Start-Sleep -Milliseconds 100
    for ($i=1; $i -le 20; $i++) {
      $x = [int]([int]$p['x1'] + ([int]$p['x2']-[int]$p['x1']) * $i / 20)
      $y = [int]([int]$p['y1'] + ([int]$p['y2']-[int]$p['y1']) * $i / 20)
      [CU.Native]::SetCursorPos($x, $y) | Out-Null; Start-Sleep -Milliseconds 25
    }
    [CU.Native]::mouse_event(0x0040,0,0,0,[UIntPtr]::Zero)
  }
  'scroll'  {
    [CU.Native]::SetCursorPos([int]$p['x'], [int]$p['y']) | Out-Null; Start-Sleep -Milliseconds 100
    $n = [Math]::Max(1, [Math]::Abs([int]$p['delta']) / 120)
    for ($i=0; $i -lt $n; $i++) { [CU.Native]::mouse_event(0x0800,0,0,[int]$p['delta'],[UIntPtr]::Zero); Start-Sleep -Milliseconds 80 }
  }
  'type'    { [System.Windows.Forms.SendKeys]::SendWait(($p['text'] -replace '\+', '{+}' -replace '\^', '{^}' -replace '%', '{%}' -replace '~', '{~}' -replace '\(', '{(}' -replace '\)', '{)}')) }
  'key'     {
    foreach ($k in $rest) {
      if ($k -match '^-') { continue }
      [System.Windows.Forms.SendKeys]::SendWait((Convert-SendKeys $k)); Start-Sleep -Milliseconds 180
    }
  }
  'uiafind' {
    Add-Type -AssemblyName UIAutomationClient
    $root = if ($p['parent']) {
      $proc = Get-TargetWindow $p['parent'] 0
      [System.Windows.Automation.AutomationElement]::FromHandle($proc.MainWindowHandle)
    } else { [System.Windows.Automation.AutomationElement]::RootElement }
    $cond = New-Object System.Windows.Automation.PropertyCondition([System.Windows.Automation.AutomationElement]::NameProperty, $p['name'])
    $els = $root.FindAll([System.Windows.Automation.TreeScope]::Descendants, $cond)
    foreach ($e in $els) {
      $r = $e.Current.BoundingRectangle
      "{0}`t{1}`t{2}" -f $e.Current.ControlType.ProgrammaticName, [int]($r.X+$r.Width/2), [int]($r.Y+$r.Height/2)
    }
  }
  'uiaclick' {
    Add-Type -AssemblyName UIAutomationClient
    $root = if ($p['parent']) {
      $proc = Get-TargetWindow $p['parent'] 0
      [System.Windows.Automation.AutomationElement]::FromHandle($proc.MainWindowHandle)
    } else { [System.Windows.Automation.AutomationElement]::RootElement }
    $cond = New-Object System.Windows.Automation.PropertyCondition([System.Windows.Automation.AutomationElement]::NameProperty, $p['name'])
    $els = $root.FindAll([System.Windows.Automation.TreeScope]::Descendants, $cond)
    if ($els.Count -eq 0) { throw "UIA element '$($p['name'])' not found" }
    $r = $els[0].Current.BoundingRectangle
    $cx = [int]($r.X + $r.Width/2); $cy = [int]($r.Y + $r.Height/2)
    & $MyInvocation.MyCommand.Path 'click' '-x' $cx '-y' $cy
    Write-Output "clicked $cx,$cy"
  }
  default { throw "unknown command '$cmd'" }
}
