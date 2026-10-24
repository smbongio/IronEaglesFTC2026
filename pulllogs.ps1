<#
.SYNOPSIS
    Copies Robot Controller logs from the robot onto this laptop over adb.

.EXAMPLE
    .\pulllogs.ps1                         # pull logs, then rotate the robot log and logcat
.EXAMPLE
    .\pulllogs.ps1 -Address 192.168.43.1   # connect to a specific address (port 5555 is added if omitted)
.EXAMPLE
    .\pulllogs.ps1 -KeepRemote             # pull without clearing the robot's log
#>
param(
    [string]$Address,
    [switch]$KeepRemote
)

# Default Control Hub address when the laptop is on the robot's Wi-Fi network.
$defaultAddress = '192.168.43.1'

$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
$remoteLog = '/sdcard/RobotControllerLog.txt'
$remoteLogAlt = '/storage/emulated/0/RobotControllerLog.txt'

function Fail($message) { Write-Host "ERROR: $message" -ForegroundColor Red; exit 1 }
function Step($message) { Write-Host "`n=== $message ===" -ForegroundColor Cyan }

function Get-AdbRows {
    @(& $adb devices) | ForEach-Object { $_.ToString().Trim() } | Where-Object { $_ -match '^\S+\s+\S+' } | ForEach-Object {
        $parts = $_ -split '\s+', 2
        [pscustomobject]@{ Serial = $parts[0]; State = (($parts[1] -split '\s+')[0]) }
    }
}

function Get-AdbDevices {
    Get-AdbRows | Where-Object { $_.State -eq 'device' }
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

    $row = Get-AdbRows | Where-Object { $_.Serial -eq $serial } | Select-Object -First 1
    if ($row -and $row.State -ne 'device') {
        Show-Adb @('reconnect', 'offline')
        Start-Sleep -Seconds 1
        Show-Adb @('connect', $serial)
        Start-Sleep -Seconds 1
    }
}

function Invoke-Adb {
    param([Parameter(ValueFromRemainingArguments = $true)][string[]]$AdbArgs)
    if ($serial) {
        & $adb -s $serial @AdbArgs
    } else {
        & $adb @AdbArgs
    }
}

function Test-RemoteFile($path) {
    $result = Invoke-Adb shell "if [ -f '$path' ]; then echo yes; else echo no; fi"
    return ($result | Out-String).Trim() -eq 'yes'
}

$localProps = Get-Content (Join-Path $root 'local.properties') -Raw -ErrorAction SilentlyContinue
if ($localProps -match 'sdk\.dir=(.+)') { $sdkDir = $Matches[1].Trim() -replace '\\:', ':' -replace '\\\\', '\' }
else { $sdkDir = "$env:LOCALAPPDATA\Android\Sdk" }
$adb = Join-Path $sdkDir 'platform-tools\adb.exe'
if (-not (Test-Path $adb)) { Fail "adb not found at $adb" }

$serial = $null
Step 'Connecting to robot'
if ($Address) {
    Connect-Adb $Address
} elseif ((Get-AdbDevices).Count -eq 0) {
    Connect-Adb $defaultAddress
}

$devices = Get-AdbDevices
if ($devices.Count -eq 0) {
    $rows = Get-AdbRows
    $detail = if ($rows) { ($rows | ForEach-Object { "$($_.Serial) $($_.State)" }) -join ', ' } else { 'none listed' }
    Fail "No robot ready (adb devices: $detail). Plug in USB, or join the Control Hub Wi-Fi and retry (tried $defaultAddress)."
}
if ($devices.Count -gt 1 -and -not $serial) {
    Fail "More than one device attached; pass -Address to pick one:`n$(($devices | ForEach-Object { $_.Serial }) -join "`n")"
}
if (-not $serial) { $serial = $devices[0].Serial }
Write-Host "Device: $serial"

$stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$dest = Join-Path $root "logs\$stamp"
New-Item -ItemType Directory -Path $dest -Force | Out-Null

Step 'Pulling RobotControllerLog'
$source = $null
if (Test-RemoteFile $remoteLog) { $source = $remoteLog }
elseif (Test-RemoteFile $remoteLogAlt) { $source = $remoteLogAlt }
if (-not $source) {
    Fail "RobotControllerLog.txt not found on the robot (tried $remoteLog and $remoteLogAlt)."
}

$localRobotLog = Join-Path $dest 'RobotControllerLog.txt'
Write-Host "adb pull $source $localRobotLog"
Invoke-Adb pull $source $localRobotLog
if ($LASTEXITCODE -ne 0 -or -not (Test-Path $localRobotLog)) { Fail 'RobotControllerLog pull failed.' }

Step 'Dumping logcat'
$localLogcat = Join-Path $dest 'logcat.txt'
Write-Host "adb logcat -d -t 3000 > $localLogcat"
$logcatLines = Invoke-Adb logcat -d -t 3000
if ($LASTEXITCODE -ne 0) { Fail 'logcat dump failed.' }
$logcatLines | Set-Content -Path $localLogcat

if (-not $KeepRemote) {
    Step 'Rotating robot logs'
    Write-Host "Clearing $source"
    Invoke-Adb shell "printf '' > '$source'" | Out-Null
    Write-Host "adb logcat -c"
    Invoke-Adb logcat -c | Out-Null
    Write-Host "Robot log and logcat buffer cleared. Next pull will only have new lines."
}

Write-Host "`nPulled logs to $dest" -ForegroundColor Green
Write-Host "Robot controller log: $localRobotLog"
Write-Host "Search for [WHEELS] or [OCTOQUAD] after a drive run."
