$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

# --- OS detection (PS 5.1 + PS 7+ compatible) ---
function Test-IsWindows {
    if ($env:OS -eq 'Windows_NT') { return $true }
    if ($PSVersionTable.PSEdition -eq 'Desktop') { return $true }
    if ($PSVersionTable.ContainsKey('Platform') -and $PSVersionTable.Platform -eq 'Win32NT') { return $true }
    return $false
}

$script:OnWindows = Test-IsWindows

$repoRoot = $PSScriptRoot
Set-Location $repoRoot

# --- Config ---
$venv32Path         = Join-Path $repoRoot ".venv32"
$requirements32File = Join-Path $repoRoot "_requirements_x32.txt"
$python32Spec       = "-3.13-32"   # adjust if your 32-bit install differs

$venv64Path         = Join-Path $repoRoot ".venv64"
$requirements64File = Join-Path $repoRoot "_requirements_x64.txt"
$python64Spec       = "-3.13"      # adjust if needed

# --- Helpers ---
function Invoke-Native {
    param([string] $Exe, [string[]] $ArgList = @())
    & $Exe @ArgList
    if ($LASTEXITCODE -ne 0) { throw "Command failed ($LASTEXITCODE): $Exe $($ArgList -join ' ')" }
}

function Get-VenvPythonPath([string] $Venv) {
    if ($script:OnWindows) { return Join-Path $Venv "Scripts/python.exe" }
    return Join-Path $Venv "bin/python"
}

function Get-ActivateScriptPath([string] $Venv) {
    if ($script:OnWindows) { return Join-Path $Venv "Scripts/Activate.ps1" }
    return Join-Path $Venv "bin/Activate.ps1"
}

function Resolve-Python {
    param([string] $WinSpec)
    if ($script:OnWindows -and (Get-Command py -ErrorAction SilentlyContinue)) {
        return @{ Exe = "py"; Args = @($WinSpec) }
    }
    foreach ($cmd in @("python3", "python")) {
        if (Get-Command $cmd -ErrorAction SilentlyContinue) {
            return @{ Exe = $cmd; Args = @() }
        }
    }
    throw "No Python found. On Ubuntu: sudo apt install python3 python3-venv python3-pip"
}

function Setup-Venv {
    param(
        [string]    $VenvPath,
        [hashtable] $PyRunner,
        [string]    $RequirementsFile
    )

    Write-Host "=== Creating/checking virtual environment '$VenvPath' ==="

    if (-not (Test-Path $VenvPath)) {
        Write-Host "Creating venv using: $($PyRunner.Exe) $($PyRunner.Args -join ' ')"
        Invoke-Native $PyRunner.Exe ($PyRunner.Args + @("-m", "venv", $VenvPath))
    } else {
        Write-Host "Venv already exists, skipping creation."
    }

    $venvPython = Get-VenvPythonPath $VenvPath
    if (-not (Test-Path $venvPython)) { throw "Venv python not found at '$venvPython'." }

    & $venvPython -m pip --version *> $null
    if ($LASTEXITCODE -ne 0) { Invoke-Native $venvPython @("-m", "ensurepip", "--upgrade") }
    Invoke-Native $venvPython @("-m", "pip", "install", "-U", "pip", "setuptools", "wheel")

    if (Test-Path $RequirementsFile) {
        Write-Host "=== Installing packages from '$RequirementsFile' ==="
        Invoke-Native $venvPython @("-m", "pip", "install", "-r", $RequirementsFile)
    } else {
        Write-Warning "Requirements file '$RequirementsFile' not found. Skipping."
    }

    $activateScript = Get-ActivateScriptPath $VenvPath
    if (Test-Path $activateScript) { . $activateScript }

    return $venvPython
}

# --- .venv32 (32-bit Python on Windows; same packages via 64-bit on Linux) ---
Write-Host ""
Write-Host "##############################"
Write-Host " Setting up .venv32 environment"
Write-Host "##############################"

$python32Exe = $null
if ($script:OnWindows) {
    if (Get-Command py -ErrorAction SilentlyContinue) {
        $py32 = Resolve-Python -WinSpec $python32Spec
        $python32Exe = Setup-Venv -VenvPath $venv32Path -PyRunner $py32 -RequirementsFile $requirements32File
        $env:PYTHON32_PATH = $python32Exe
        Write-Host "PYTHON32_PATH set to '$python32Exe'."
    } else {
        Write-Warning "'py' launcher not found -- skipping .venv32 (install Python 3.13 32-bit + py launcher)."
    }
} else {
    # On Linux the SPM-002 DLL cannot be loaded, but the subprocess can still run.
    # Use the system 64-bit Python for the .venv32 environment.
    Write-Host "(Linux: using 64-bit Python for .venv32 -- DLL loading is skipped on Linux)"
    $py32 = Resolve-Python -WinSpec $python64Spec
    $python32Exe = Setup-Venv -VenvPath $venv32Path -PyRunner $py32 -RequirementsFile $requirements32File
}

# --- .venv64 ---
Write-Host ""
Write-Host "##############################"
Write-Host " Setting up .venv64 environment"
Write-Host "##############################"

$py64 = Resolve-Python -WinSpec $python64Spec
$python64Exe = Setup-Venv -VenvPath $venv64Path -PyRunner $py64 -RequirementsFile $requirements64File
Write-Host "64-bit python: '$python64Exe'"

Write-Host ""
Write-Host "=== Done. .venv64 '$venv64Path' is active in this session. ==="
if ($python32Exe) {
    Write-Host ".venv32 python (SPM-002 subprocess): '$python32Exe'$(if ($env:PYTHON32_PATH) { " (`$env:PYTHON32_PATH)" })"
}
