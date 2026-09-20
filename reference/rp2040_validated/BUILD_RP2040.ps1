$ErrorActionPreference = "Stop"

# ================================================================
# BUILD_RP2040.ps1
# Clean build for Raspberry Pi Pico 1 / RP2040
# ================================================================

$ProjectDir = Split-Path -Parent $MyInvocation.MyCommand.Path

$SdkDir       = "C:\Users\Juan\.pico-sdk\sdk\2.3.0"
$ToolchainDir = "C:\Users\Juan\.pico-sdk\toolchain\15_2_Rel1"
$NinjaDir     = "C:\Users\Juan\.pico-sdk\ninja\v1.13.2"

$Gcc = Join-Path $ToolchainDir "bin\arm-none-eabi-gcc.exe"
$Gxx = Join-Path $ToolchainDir "bin\arm-none-eabi-g++.exe"
$Ninja = Join-Path $NinjaDir "ninja.exe"

if (!(Test-Path $SdkDir)) {
    throw "Pico SDK not found: $SdkDir"
}
if (!(Test-Path $Gcc)) {
    throw "ARM GCC not found: $Gcc"
}
if (!(Test-Path $Gxx)) {
    throw "ARM G++ not found: $Gxx"
}
if (!(Test-Path $Ninja)) {
    throw "Ninja not found: $Ninja"
}

# Make the official SDK compiler discovery work even if VS Code
# has not imported the folder as a Pico project.
$env:PICO_SDK_PATH = $SdkDir
$env:PICO_TOOLCHAIN_PATH = $ToolchainDir
$env:Path = "$($ToolchainDir)\bin;$NinjaDir;$env:Path"

Write-Host ""
Write-Host "============================================================"
Write-Host " PICO 1 / RP2040 - CLEAN BUILD"
Write-Host "============================================================"
Write-Host "SDK       : $SdkDir"
Write-Host "Toolchain : $ToolchainDir"
Write-Host "Ninja     : $Ninja"
Write-Host ""

Push-Location $ProjectDir
try {
    if (Test-Path ".\build") {
        Write-Host "Removing old build..."
        Remove-Item -Recurse -Force ".\build"
    }

    Write-Host ""
    Write-Host "[1/2] Configuring CMake..."

    # Explicit compiler paths are also passed as a fallback.
    & cmake `
        -S . `
        -B build `
        -G Ninja `
        "-DCMAKE_MAKE_PROGRAM=$Ninja" `
        "-DPICO_SDK_PATH=$($SdkDir -replace '\\','/')" `
        "-DPICO_TOOLCHAIN_PATH=$($ToolchainDir -replace '\\','/')" `
        "-DPICO_BOARD=pico" `
        "-DPICO_PLATFORM=rp2040" `
        "-DPICO_COMPILER=pico_arm_gcc" `
        "-DCMAKE_C_COMPILER=$($Gcc -replace '\\','/')" `
        "-DCMAKE_CXX_COMPILER=$($Gxx -replace '\\','/')" `
        "-DCMAKE_ASM_COMPILER=$($Gcc -replace '\\','/')" `
        "-DCMAKE_BUILD_TYPE=Release" `
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"

    if ($LASTEXITCODE -ne 0) {
        throw "CMake configuration failed with exit code $LASTEXITCODE"
    }

    Write-Host ""
    Write-Host "[2/2] Compiling..."

    & cmake --build build --parallel 4

    if ($LASTEXITCODE -ne 0) {
        throw "Build failed with exit code $LASTEXITCODE"
    }

    $Uf2 = Join-Path $ProjectDir "build\glove_ukf_shadow.uf2"

    Write-Host ""
    Write-Host "============================================================"
    if (Test-Path $Uf2) {
        Write-Host " BUILD OK"
        Write-Host " UF2: $Uf2"
    } else {
        Write-Host " Build finished, but UF2 was not found at:"
        Write-Host " $Uf2"
    }
    Write-Host "============================================================"
}
finally {
    Pop-Location
}
