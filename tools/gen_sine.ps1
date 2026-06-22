$ARR = 2000
$MID = 1000
$SWING = 900
$N = 256
$norm = 0.8660254037844386   # max of sin(x) + sin(3x)/6

$vals = [System.Collections.ArrayList]::new()

for ($i = 0; $i -lt $N; $i++) {
    $th = 2.0 * [Math]::PI * $i / $N
    $v = [Math]::Sin($th) + [Math]::Sin(3.0 * $th) / 6.0
    $ccr = [int]([Math]::Round($MID + $SWING * $v / $norm))
    if ($ccr -lt 0) { $ccr = 0 }
    if ($ccr -gt $ARR) { $ccr = $ARR }
    [void]$vals.Add($ccr)
}

Write-Output "static const uint16_t sin3_table[256] = {"
for ($r = 0; $r -lt 16; $r++) {
    $start = $r * 16
    $lineVals = $vals[$start..($start + 15)] -join ", "
    Write-Output "    $lineVals,"
}
Write-Output "};"
