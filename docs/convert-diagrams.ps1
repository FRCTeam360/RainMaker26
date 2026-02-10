# Convert all Mermaid diagrams to SVG
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

Get-ChildItem -Path $scriptDir -Filter "*.mmd" | ForEach-Object {
    $inputFile = $_.FullName
    $outputFile = [System.IO.Path]::ChangeExtension($inputFile, ".svg")

    Write-Host "Converting $($_.Name) -> $([System.IO.Path]::GetFileName($outputFile))"
    mmdc -i $inputFile -o $outputFile
}

Write-Host "Done!"
