<#
  keil-build.ps1  —  用 Keil 命令行（UV4.exe）编译 STM32 工程
  供 VS Code 任务调用，也可以直接在终端里手动跑：

      powershell -ExecutionPolicy Bypass -File tools\keil-build.ps1 -Mode build
      powershell -ExecutionPolicy Bypass -File tools\keil-build.ps1 -Mode rebuild
      powershell -ExecutionPolicy Bypass -File tools\keil-build.ps1 -Mode clean

  UV4 的退出码：
      0 = 无错误无警告      1 = 成功但有警告
      2 = 有错误            3 = 致命错误
#>
param(
    [ValidateSet('build', 'rebuild', 'clean')]
    [string]$Mode = 'build'
)

# 让终端能正确显示 UTF-8（Keil 日志里可能有中文路径）
try { [Console]::OutputEncoding = [Text.Encoding]::UTF8 } catch { }

$ErrorActionPreference = 'Continue'

# 脚本在 <工作区根>\tools\ 下，往上一级就是工作区根
$root = Split-Path -Parent $PSScriptRoot
$proj = Join-Path $root 'sentry\MDK-ARM\sentry.uvprojx'
$log = Join-Path $root 'sentry\MDK-ARM\build.log'
$uv = 'C:\Keil_v5\UV4\UV4.exe'

foreach ($p in @($uv, $proj)) {
    if (-not (Test-Path $p)) {
        Write-Host "[keil-build] 找不到: $p" -ForegroundColor Red
        exit 3
    }
}

Remove-Item $log -ErrorAction SilentlyContinue

$flag = switch ($Mode) {
    'build' { '-b' }
    'rebuild' { '-r' }
    'clean' { '-c' }
}

# UV4 把编译输出写进 -o 指定的日志，不写 stdout，所以跑完要自己读出来
& $uv $flag $proj -j0 -o $log | Out-Null
$code = $LASTEXITCODE

if (Test-Path $log) {
    Get-Content $log
}

Write-Host ''
switch ($code) {
    0 { Write-Host '===== BUILD OK: 0 errors, 0 warnings =====' -ForegroundColor Green }
    1 { Write-Host '===== BUILD OK: with warnings ===========' -ForegroundColor Yellow }
    2 { Write-Host '===== BUILD FAILED: errors ==============' -ForegroundColor Red }
    3 { Write-Host '===== BUILD FAILED: fatal ===============' -ForegroundColor Red }
    default { Write-Host "===== UV4 exit code $code ================" -ForegroundColor Red }
}

# 0/1 都算成功（VS Code 任务把非 0 当失败，这里统一一下）
if ($code -le 1) { exit 0 } else { exit 1 }
