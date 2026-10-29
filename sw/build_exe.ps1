#!/usr/bin/env pwsh
<#
  build_exe.ps1 — 把 uart2axi_gui.py 打包成 Windows 可执行文件（PyInstaller）

  用法（在本目录打开 PowerShell）:
    .\build_exe.ps1                  # 单文件：dist\uart2axi_gui.exe（一个文件就能发人）
    .\build_exe.ps1 -OneDir          # 目录版：dist\uart2axi_gui\（启动更快、杀软误报更少）
    .\build_exe.ps1 -Icon app.ico    # 顺便指定 exe 图标
    .\build_exe.ps1 -Console         # 带控制台窗口，只看报错用（正式分发别用）

  说明:
    * 首次打包前需要 PyInstaller:  uv pip install pyinstaller
    * 目标机器不需要装 Python，也没有额外运行库要求（Win10/11 自带）
    * 产物在 dist\ 下；build\ 是中间文件，可随时删除

  许可:
    本仓库为 ultraembedded/dbg_bridge 的非官方修改版，整体以
    LGPL-2.1-or-later 发布，见仓库根目录的 LICENSE 与 NOTICE。
    分发 exe 时请同时提供完整源码（本仓库）以便接收者可自行重新构建。
#>
param(
    [switch]$OneDir,           # 打成目录而不是单文件
    [switch]$Console,          # 保留控制台窗口（排查启动报错）
    [string]$Icon  = ''        # 可选 .ico 图标
)

$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot

# 优先用项目自带的虚拟环境，没有就退回 PATH 里的 python
$python = Join-Path $PSScriptRoot '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $python)) {
    $python = 'python'
}

& $python -c "import PyInstaller" 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Host '未检测到 PyInstaller，先执行: uv pip install pyinstaller' -ForegroundColor Yellow
    exit 1
}

$pyiArgs = @(
    '-m', 'PyInstaller',
    '--noconfirm', '--clean',           # 覆盖旧产物，不带缓存
    '--noupx',
    '--name', 'uart2axi_gui',
    '--hidden-import', 'serial.tools.list_ports',
    '--distpath', 'dist',
    '--workpath', 'build',
    '--specpath', 'build'
)
if (-not $Console) { $pyiArgs += '--windowed' }   # 默认不弹黑色控制台窗口
if ($OneDir) { $pyiArgs += '--onedir' } else { $pyiArgs += '--onefile' }
if ($Icon)   { $pyiArgs += @('--icon', $Icon) }
$pyiArgs += 'uart2axi_gui.py'

Write-Host ('打包中: ' + $python + ' ' + ($pyiArgs -join ' ')) -ForegroundColor Cyan
& $python @pyiArgs
if ($LASTEXITCODE -ne 0) {
    Write-Host '打包失败，请查看上面的报错。' -ForegroundColor Red
    exit $LASTEXITCODE
}

if ($OneDir) {
    $exe = Join-Path $PSScriptRoot 'dist\uart2axi_gui\uart2axi_gui.exe'
} else {
    $exe = Join-Path $PSScriptRoot 'dist\uart2axi_gui.exe'
}
if (Test-Path -LiteralPath $exe) {
    $size = [math]::Round((Get-Item -LiteralPath $exe).Length / 1MB, 1)
    Write-Host ''
    Write-Host ('完成: ' + $exe + '  (' + $size + ' MB)') -ForegroundColor Green
    if ($OneDir) {
        Write-Host '发人时请把整个 dist\uart2axi_gui\ 目录压缩成 zip。'
    } else {
        Write-Host '单文件首次启动会解压到临时目录，慢 1~3 秒属正常现象。'
    }
} else {
    Write-Host '没有找到产物，请检查上面的打包日志。' -ForegroundColor Red
    exit 1
}