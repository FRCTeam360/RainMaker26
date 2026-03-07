# Convert all Mermaid diagrams to PNG recursively
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$rootDir = Split-Path -Parent $scriptDir

Get-ChildItem -Path $rootDir -Filter "*.mmd" -Recurse | ForEach-Object {
    $inputFile = $_.FullName
    $outputFile = [System.IO.Path]::ChangeExtension($inputFile, ".png")

    Write-Host "Converting $($_.FullName) -> $([System.IO.Path]::GetFileName($outputFile))"
    mmdc -i $inputFile -o $outputFile --scale 4
}

Write-Host "Done!"
