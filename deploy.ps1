<#
.SYNOPSIS
    Builds TeamCode and installs/launches the Robot Controller app on the robot over adb.

.EXAMPLE
    .\deploy.ps1                         # USB if already attached, otherwise adb connect to the Control Hub Wi-Fi
.EXAMPLE
    .\deploy.ps1 -Address 192.168.43.1   # connect to a specific address (port 5555 is added if omitted)
.EXAMPLE
    .\deploy.ps1 -BuildOnly              # compile and package without a robot
#>
param(
    [string]$Address,
    [switch]$BuildOnly
)

# Default Control Hub address when the laptop is on the robot's Wi-Fi network.
$defaultAddress = '192.168.43.1'

$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
$appId = 'com.qualcomm.ftcrobotcontroller'
$launchActivity = 'org.firstinspires.ftc.robotcontroller.internal.PermissionValidatorWrapper'
$apk = Join-Path $root 'TeamCode\build\outputs\apk\debug\TeamCode-debug.apk'

function Fail($message) { Write-Host "ERROR: $message" -ForegroundColor Red; exit 1 }
function Step($message) { Write-Host "`n=== $message ===" -ForegroundColor Cyan }

function Get-AdbDevices {
    @(& $adb devices | Select-Object -Skip 1 | Where-Object { $_ -match '\tdevice$' })
}

function Connect-Adb($hostAddress) {
    $script:serial = if ($hostAddress -match ':') { $hostAddress } else { "${hostAddress}:5555" }
    Write-Host "adb connect $serial"
    # adb prints to stdout; keep that out of the function's return value so it is not passed as extra args later.
    & $adb connect $serial | ForEach-Object { Write-Host $_ }
}

function Invoke-Adb {
    param([Parameter(ValueFromRemainingArguments = $true)][string[]]$AdbArgs)
    if ($serial) {
        & $adb -s $serial @AdbArgs
    } else {
        & $adb @AdbArgs
    }
}

# Gradle 8.7 cannot run on Android Studio's bundled Java 25, so use the JDK 21 the project is configured for.
$jdk = Get-ChildItem "$env:USERPROFILE\.jdks" -Directory -Filter 'jbr-21*' -ErrorAction SilentlyContinue |
    Where-Object { Test-Path "$($_.FullName)\bin\java.exe" } | Select-Object -First 1
if ($jdk) { $env:JAVA_HOME = $jdk.FullName }
elseif (-not $env:JAVA_HOME) { Fail 'No JDK 21 found in ~\.jdks and JAVA_HOME is not set.' }

# The repo has no gradle-wrapper.jar, so run the Gradle that Android Studio downloaded for this project.
$wrapperProps = Get-Content (Join-Path $root 'gradle\wrapper\gradle-wrapper.properties') -Raw
if ($wrapperProps -notmatch 'gradle-([\d.]+)-bin\.zip') { Fail 'Could not read the Gradle version from gradle-wrapper.properties.' }
$gradleVersion = $Matches[1]
$gradle = Get-ChildItem "$env:USERPROFILE\.gradle\wrapper\dists\gradle-$gradleVersion-bin" -Recurse -Filter gradle.bat -ErrorAction SilentlyContinue |
    Select-Object -First 1
if (-not $gradle) { Fail "Gradle $gradleVersion not found. Open the project in Android Studio once so it downloads it." }

$localProps = Get-Content (Join-Path $root 'local.properties') -Raw -ErrorAction SilentlyContinue
if ($localProps -match 'sdk\.dir=(.+)') { $sdkDir = $Matches[1].Trim() -replace '\\:', ':' -replace '\\\\', '\' }
else { $sdkDir = "$env:LOCALAPPDATA\Android\Sdk" }
$adb = Join-Path $sdkDir 'platform-tools\adb.exe'

$serial = $null
if (-not $BuildOnly) {
    if (-not (Test-Path $adb)) { Fail "adb not found at $adb" }

    Step 'Connecting to robot'
    if ($Address) {
        Connect-Adb $Address
    } elseif ((Get-AdbDevices).Count -eq 0) {
        Connect-Adb $defaultAddress
    }

    $devices = Get-AdbDevices
    if ($devices.Count -eq 0) {
        Fail "No robot found. Plug in USB, or join the Control Hub Wi-Fi and retry (tried $defaultAddress)."
    }
    if ($devices.Count -gt 1 -and -not $serial) {
        Fail "More than one device attached; pass -Address to pick one:`n$($devices -join "`n")"
    }
    Write-Host "Device: $(($devices | Select-Object -First 1) -replace '\t.*')"
}

Step "Building with Gradle $gradleVersion"
Push-Location $root
try {
    # Trust the Windows certificate store, as Android Studio does, so HTTPS downloads work behind antivirus/proxy inspection.
    & $gradle.FullName ':TeamCode:assembleDebug' '-Djavax.net.ssl.trustStoreType=Windows-ROOT' '--console=plain'
    if ($LASTEXITCODE -ne 0) { Fail 'Build failed.' }
} finally {
    Pop-Location
}
if (-not (Test-Path $apk)) { Fail "Build succeeded but the APK is missing: $apk" }
if ($BuildOnly) { Write-Host "`nBuilt $apk" -ForegroundColor Green; exit 0 }

Step 'Installing'
Invoke-Adb install -r $apk
if ($LASTEXITCODE -ne 0) { Fail 'Install failed.' }

Step 'Launching Robot Controller'
Invoke-Adb shell am start -S -n "$appId/$launchActivity"
if ($LASTEXITCODE -ne 0) { Fail 'Launch failed.' }

Write-Host "`nDeployed." -ForegroundColor Green
