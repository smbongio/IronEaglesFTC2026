<#
.SYNOPSIS
    Copies IronEaglesConfig.xml to the Control Hub over adb and restarts the Robot Controller.

.EXAMPLE
    .\pushconfig.ps1                         # USB if already attached, otherwise adb connect to the Control Hub Wi-Fi
.EXAMPLE
    .\pushconfig.ps1 -Address 192.168.43.1   # connect to a specific address (port 5555 is added if omitted)
#>
param(
    [string]$Address
)

# Default Control Hub address when the laptop is on the robot's Wi-Fi network.
$defaultAddress = '192.168.43.1'

$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
$appId = 'com.qualcomm.ftcrobotcontroller'
$launchActivity = 'org.firstinspires.ftc.robotcontroller.internal.PermissionValidatorWrapper'
$configName = 'IronEaglesConfig.xml'
$localConfig = Join-Path $root $configName
# FTC stores Driver Station robot configs in /sdcard/FIRST on the Robot Controller.
$remoteConfig = "/sdcard/FIRST/$configName"

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

if (-not (Test-Path $localConfig)) { Fail "Hardware config not found: $localConfig" }

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

Step "Pushing $configName"
Write-Host "adb push $localConfig $remoteConfig"
# Quote the remote command so adb does not treat -p as its own product flag.
Invoke-Adb shell "mkdir -p /sdcard/FIRST"
if ($LASTEXITCODE -ne 0) { Fail "Could not create /sdcard/FIRST on the robot." }
Invoke-Adb push $localConfig $remoteConfig
if ($LASTEXITCODE -ne 0) { Fail 'Config push failed.' }

Step 'Launching Robot Controller'
Invoke-Adb shell am start -S -n "$appId/$launchActivity"
if ($LASTEXITCODE -ne 0) { Fail 'Launch failed.' }

Write-Host "`nPushed $configName to $remoteConfig" -ForegroundColor Green
Write-Host "If this is a new config name, Activate 'IronEaglesConfig' on the Driver Station (Configure Robot)."
