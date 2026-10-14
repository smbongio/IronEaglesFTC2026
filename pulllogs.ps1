<#
.SYNOPSIS
    Copies Robot Controller logs from the robot onto this laptop over adb.

.EXAMPLE
    .\pulllogs.ps1                         # USB if attached, otherwise Control Hub Wi-Fi
.EXAMPLE
    .\pulllogs.ps1 -Address 192.168.43.1   # specific address
#>
param(
    [string]$Address
)

$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
$defaultAddress = '192.168.43.1'
$remoteLogs = @(
    '/sdcard/RobotControllerLog.txt',
    '/storage/emulated/0/RobotControllerLog.txt'
)

function Fail($message) { Write-Host "ERROR: $message" -ForegroundColor Red; exit 1 }
function Step($message) { Write-Host "`n=== $message ===" -ForegroundColor Cyan }

$localProps = Get-Content (Join-Path $root 'local.properties') -Raw -ErrorAction SilentlyContinue
if ($localProps -match 'sdk\.dir=(.+)') { $sdkDir = $Matches[1].Trim() -replace '\\:', ':' -replace '\\\\', '\' }
else { $sdkDir = "$env:LOCALAPPDATA\Android\Sdk" }
$adb = Join-Path $sdkDir 'platform-tools\adb.exe'
if (-not (Test-Path $adb)) { Fail "adb not found at $adb" }

function Get-AdbRows {
    @(& $adb devices) | ForEach-Object { $_.ToString().Trim() } | Where-Object { $_ -match '^\S+\s+\S+' } | ForEach-Object {
        $parts = $_ -split '\s+', 2
        [pscustomobject]@{ Serial = $parts[0]; State = (($parts[1] -split '\s+')[0]) }
    }
}

function Show-Adb($AdbArgs) {
    Write-Host "adb $($AdbArgs -join ' ')"
    & $adb @AdbArgs | ForEach-Object { Write-Host $_ }
}

function Connect-Adb($hostAddress) {
    $script:serial = if ($hostAddress -match ':') { $hostAddress } else { "${hostAddress}:5555" }
    Show-Adb @('connect', $serial)
    $row = Get-AdbRows | Where-Object { $_.Serial -eq $serial } | Select-Object -First 1
    if ($row -and $row.State -eq 'device') { return }

    Write-Host "Device state is $(if ($row) { $row.State } else { 'missing' }); reconnecting."
    Show-Adb @('disconnect', $serial)
    Start-Sleep -Seconds 1
    Show-Adb @('connect', $serial)
    Start-Sleep -Seconds 1
}

$serial = $null
Step 'Connecting to robot'
if ($Address) {
    Connect-Adb $Address
} elseif (-not (Get-AdbRows | Where-Object { $_.State -eq 'device' })) {
    Connect-Adb $defaultAddress
}

$ready = @(Get-AdbRows | Where-Object { $_.State -eq 'device' })
if ($ready.Count -eq 0) {
    Fail "No robot ready. Plug in USB (keeps laptop internet), or join the Control Hub Wi-Fi and retry."
}
if (-not $serial) { $serial = $ready[0].Serial }
Write-Host "Device: $serial"

$stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$outDir = Join-Path $root "logs\$stamp"
New-Item -ItemType Directory -Force -Path $outDir | Out-Null

Step 'Copying Robot Controller log'
$copied = $false
foreach ($remote in $remoteLogs) {
    $local = Join-Path $outDir 'RobotControllerLog.txt'
    & $adb -s $serial pull $remote $local
    if ($LASTEXITCODE -eq 0 -and (Test-Path $local)) {
        $copied = $true
        break
    }
}
if (-not $copied) {
    Write-Host "RobotControllerLog.txt was not on the hub. Dumping logcat only."
}

Step 'Dumping logcat'
$logcat = Join-Path $outDir 'logcat.txt'
& $adb -s $serial logcat -d | Out-File -FilePath $logcat -Encoding utf8

Write-Host "`nSaved to $outDir" -ForegroundColor Green
Get-ChildItem $outDir | ForEach-Object { Write-Host ("  {0}  {1:N0} bytes" -f $_.Name, $_.Length) }
Write-Host "Search the files for [VISION] after you are back on internet Wi-Fi."
