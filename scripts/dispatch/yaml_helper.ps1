param(
    [Parameter(Mandatory=$true)][string]$File,
    [Parameter(Mandatory=$true)][string]$Section,
    [string]$ListKey,
    [switch]$ReturnList
)

try {
    $lines = Get-Content -Path $File -ErrorAction Stop
} catch {
    return
}

if (-not $lines) {
    return
}

$sectionPattern = '^[\s]*' + [regex]::Escape($Section) + ':\s*$'
$sectionMatch = $null
for ($i = 0; $i -lt $lines.Count; $i++) {
    if ($lines[$i] -match $sectionPattern) {
        $sectionMatch = $i
        break
    }
}

if ($null -eq $sectionMatch) {
    return
}

$sectionIndent = $lines[$sectionMatch].Length - $lines[$sectionMatch].TrimStart().Length

if ($ReturnList) {
    if (-not $ListKey) { return }
    $listPattern = '^[\s]*' + [regex]::Escape($ListKey) + ':\s*$'
    $listIndent = $null
    $values = @()
    for ($j = $sectionMatch + 1; $j -lt $lines.Count; $j++) {
        $line = $lines[$j]
        if ([string]::IsNullOrWhiteSpace($line)) {
            if ($null -ne $listIndent) { break } else { continue }
        }
        $indent = $line.Length - $line.TrimStart().Length
        if ($indent -le $sectionIndent) { break }
        if ($null -eq $listIndent) {
            if ($line -match $listPattern) {
                $listIndent = $indent
            }
        } else {
            if ($indent -le $listIndent) { break }
            $trimmed = $line.Trim()
            if ($trimmed -like '-*') {
                $value = $trimmed -replace '^-\s*', ''
                $value = ($value -split '\s+#')[0]
                $value = $value.Trim('"')
                if ($value) {
                    $values += $value
                }
            } else {
                break
            }
        }
    }
    if ($values.Count -gt 0) {
        Write-Output ($values -join ' ')
    }
} else {
    for ($j = $sectionMatch + 1; $j -lt $lines.Count; $j++) {
        $line = $lines[$j]
        if ([string]::IsNullOrWhiteSpace($line)) { continue }
        $indent = $line.Length - $line.TrimStart().Length
        if ($indent -le $sectionIndent) { break }
        $match = [regex]::Match($line, '^[\s]*version:[\s]*"(.+?)"')
        if ($match.Success) {
            Write-Output $match.Groups[1].Value
            break
        }
    }
}
