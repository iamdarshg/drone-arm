---
name: computer-use
description: Drive Windows GUI apps (KiCad, browsers, dialogs) from opencode - take screenshots, click, drag, type, press keys, scroll, find/click UI elements. Use whenever the user asks to use the computer, control the GUI, automate a desktop app, "computer use", or interact with on-screen windows.
---

# Computer Use (Windows GUI automation)

All control flows through one PowerShell script: `scripts/cu.ps1` (relative to this SKILL.md).
Run it via the bash tool. Screenshots are PNG files — view them with the Read tool.

Base path: `D:\CodeProjects\drone-arm\.opencode\skills\computer-use\scripts\cu.ps1`

## Commands

```powershell
$cu = 'D:\CodeProjects\drone-arm\.opencode\skills\computer-use\scripts\cu.ps1'

# See what's on screen (returns PNG path -> Read that file to SEE it)
powershell -File $cu shot
powershell -File $cu shot -Out D:\CodeProjects\drone-arm\tmp\s1.png
powershell -File $cu shot -Window 'PCB Editor'

# List windows: "<pid>\t<title>\t(l,t)-(r,b)"
powershell -File $cu windows

# Focus a window before typing/keys
powershell -File $cu activate -Title 'PCB Editor'

# Mouse (screen pixel coords; hover matters in KiCad)
powershell -File $cu move   -X 900 -Y 500
powershell -File $cu click  -X 900 -Y 500            # left click
powershell -File $cu click  -X 900 -Y 500 -Double    # double click
powershell -File $cu click  -X 900 -Y 500 -Right     # right click
powershell -File $cu click  -X 900 -Y 500 -Mid       # middle click
powershell -File $cu drag   -X1 100 -Y1 100 -X2 400 -Y2 400
powershell -File $cu mdrag  -X1 900 -Y1 500 -X2 700 -Y2 600   # middle-drag pan
powershell -File $cu scroll -X 900 -Y 500 -Delta -360         # negative = down

# Keyboard (focus target first). Multiple combos allowed.
powershell -File $cu key ctrl+s
powershell -File $cu key enter esc
powershell -File $cu type -Text 'hello world'
```

## UI element discovery (dialogs/buttons)

```powershell
# Find by accessible name -> prints "ControlType\tx\ty"
powershell -File $cu uiafind -Name 'OK' -Parent 'PCB Editor'
powershell -File $cu uiaclick -Name 'OK' -Parent 'PCB Editor'   # find + click center
```

## Workflow rules

1. ALWAYS `shot` first, Read the PNG, decide coordinates from what you actually see.
2. Screen coords are physical pixels (script sets DPI-aware). Window rect comes from `windows`.
3. After each action that changes UI state, re-screenshot to verify effect.
4. For modal dialogs prefer `uiaclick` over raw coordinates.
5. Keep screenshots in `tmp/` of the project.
