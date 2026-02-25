param(
    [string]$InputDir = "docs",
    [string]$OutputDir = "docs/build",
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"

if (-not (Get-Command typst -ErrorAction SilentlyContinue)) {
    Write-Error "Typst CLI not found. Install Typst and ensure 'typst' is in PATH."
}

if (-not (Test-Path -Path $InputDir)) {
    Write-Error "Input directory not found: $InputDir"
}

$root = (Resolve-Path $InputDir).Path
$typFiles = Get-ChildItem -Path $root -Recurse -File -Filter *.typ

if ($typFiles.Count -eq 0) {
    Write-Host "No .typ files found under '$InputDir'."
    exit 0
}

Write-Host "Found $($typFiles.Count) Typst files."

foreach ($file in $typFiles) {
    $relative = $file.FullName.Substring($root.Length).TrimStart('\', '/')
    $pdfRelative = [System.IO.Path]::ChangeExtension($relative, ".pdf")
    $pdfOut = Join-Path $OutputDir $pdfRelative
    $pdfDir = Split-Path -Parent $pdfOut

    if (-not (Test-Path -Path $pdfDir)) {
        New-Item -ItemType Directory -Path $pdfDir -Force | Out-Null
    }

    $cmd = "typst compile --root `"$root`" `"$($file.FullName)`" `"$pdfOut`""
    if ($DryRun) {
        Write-Host "[DRY-RUN] $cmd"
        continue
    }

    Write-Host "[BUILD] $relative -> $pdfOut"
    typst compile --root "$root" "$($file.FullName)" "$pdfOut"
}

Write-Host "Done. PDFs generated in '$OutputDir'."
