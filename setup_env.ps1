$ErrorActionPreference = "Stop"

# Always work relative to the location of this script
$repoRoot = $PSScriptRoot
Set-Location $repoRoot

# -------------------------------------------------------------------
# Paths and files
# -------------------------------------------------------------------

# 32-bit venv (for SPM-002 acquisition, loads 32-bit DLL)
$venv32Path         = Join-Path $repoRoot ".venv32"
$requirements32File = Join-Path $repoRoot "spm-002\_requirements_x32.txt"
$python32Spec       = "-3.13-32"   # adjust if your 32-bit version is different

# 64-bit venv (optional: for dev/tests of elliptec etc.)
$venv64Path         = Join-Path $repoRoot ".venv64"
$requirements64File = Join-Path $repoRoot "spm-002\_requirements_x64.txt"
$python64Spec       = "-3.13"      # adjust if needed

# Optional VS Code extensions file (one per repo)
$extensionsFile     = Join-Path $repoRoot "_extensions.txt"

# -------------------------------------------------------------------
# Helper function
# -------------------------------------------------------------------
function Setup-Venv {
    param(
        [Parameter(Mandatory = $true)]
        [string] $VenvPath,

        [Parameter(Mandatory = $true)]
        [string] $PythonSpec,

        [Parameter(Mandatory = $true)]
        [string] $RequirementsFile
    )

    Write-Host "=== Creating/checking virtual environment '$VenvPath' ==="

    if (-not (Test-Path $VenvPath)) {
        Write-Host "Creating virtual environment with 'py $PythonSpec -m venv'..."
        & py $PythonSpec -m venv $VenvPath
    } else {
        Write-Host "Virtual environment already exists, skipping creation."
    }

    # Activate
    $activateScript = Join-Path $VenvPath "Scripts\Activate.ps1"
    if (-not (Test-Path $activateScript)) {
        throw "Activate script not found at '$activateScript'."
    }

    Write-Host "=== Activating virtual environment '$VenvPath' ==="
    & $activateScript
    Write-Host "Virtual env: $env:VIRTUAL_ENV"

    # Upgrade pip
    Write-Host "=== Upgrading pip in '$VenvPath' ==="
    python -m pip install --upgrade pip

    # Install requirements
    if (Test-Path $RequirementsFile) {
        Write-Host "=== Installing Python packages from '$RequirementsFile' ==="
        python -m pip install -r $RequirementsFile
    } else {
        Write-Warning "Requirements file '$RequirementsFile' not found. Skipping package install."
    }
}

# -------------------------------------------------------------------
# 1) Setup 32-bit environment (SPM-002 acquisition)
# -------------------------------------------------------------------
Write-Host "##############################"
Write-Host " Setting up 32-bit environment"
Write-Host "##############################"

Setup-Venv -VenvPath $venv32Path -PythonSpec $python32Spec -RequirementsFile $requirements32File

# Expose PYTHON32_PATH for convenience in this session
$python32Exe = Join-Path $venv32Path "Scripts\python.exe"
$python32Exe = (Resolve-Path $python32Exe).Path
$env:PYTHON32_PATH = $python32Exe
Write-Host "PYTHON32_PATH set to '$python32Exe'."

# -------------------------------------------------------------------
# 2) Setup 64-bit environment (optional dev/tests)
# -------------------------------------------------------------------
Write-Host ""
Write-Host "##############################"
Write-Host " Setting up 64-bit environment"
Write-Host "##############################"

Setup-Venv -VenvPath $venv64Path -PythonSpec $python64Spec -RequirementsFile $requirements64File

# The last activated venv is the 64-bit one
$python64Exe = Join-Path $venv64Path "Scripts\python.exe"
$python64Exe = (Resolve-Path $python64Exe).Path
Write-Host "64-bit python at '$python64Exe'."

# -------------------------------------------------------------------
# 3) Install VS Code extensions (optional)
# -------------------------------------------------------------------
Write-Host ""
Write-Host "=== Installing VS Code extensions (if any) ==="

if (Test-Path $extensionsFile) {
    if (-not (Get-Command code -ErrorAction SilentlyContinue)) {
        Write-Warning "'code' CLI (VS Code) not found in PATH. Skipping extension install."
    } else {
        Get-Content $extensionsFile | ForEach-Object {
            $ext = $_.Trim()
            if ($ext -and -not $ext.StartsWith("#")) {
                Write-Host "Installing VS Code extension '$ext'..."
                code --install-extension $ext
            }
        }
    }
} else {
    Write-Warning "Extensions file '$extensionsFile' not found. Skipping VS Code extension install."
}

Write-Host ""
Write-Host "=== Devices setup finished. 64-bit venv '$venv64Path' is active in this PowerShell session. ==="
Write-Host "The 32-bit python for SPM-002 is '$python32Exe' (also in \$env:PYTHON32_PATH)."
