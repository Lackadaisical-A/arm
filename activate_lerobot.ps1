$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$venvActivate = Join-Path $scriptDir ".venv\Scripts\Activate.ps1"
$ffmpegBin = Join-Path $scriptDir "tools\ffmpeg\ffmpeg-n8.1-latest-win64-lgpl-shared-8.1\bin"

. $venvActivate

if (Test-Path $ffmpegBin) {
    $pathParts = $env:Path -split ";"
    if ($pathParts -notcontains $ffmpegBin) {
        $env:Path = "$ffmpegBin;$env:Path"
    }
}

Write-Host "LeRobot venv active. Local FFmpeg is on PATH for this PowerShell session."
