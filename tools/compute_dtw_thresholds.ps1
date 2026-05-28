param(
    [string]$SessionFile = ".\calibration\raw\session_example.txt",
    [string]$ReportOut = ".\calibration\dtw_threshold_report.csv",
    [string]$TemplateOut = ".\calibration\dtw_templates.json",
    [string]$HeaderOut = ".\main\DTWCalibrationData.h",
    [double]$FlexScale = 90.0,
    [double]$ThresholdMultiplier = 1.50,
    [double]$SmallSampleMultiplier = 2.00,
    [int]$SmallSampleMax = 3,
    [int]$MaxTemplateFrames = 25,
    [int]$MaxTemplatesPerAction = 4,
    [switch]$UseRelativeAngles
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
$Utf8NoBom = New-Object System.Text.UTF8Encoding($false)

function Parse-Double([string]$text) {
    return [double]::Parse($text.Trim(), [System.Globalization.CultureInfo]::InvariantCulture)
}

function New-TrialKey([string]$action, [string]$trialId) {
    return "$action#$trialId"
}

function Get-FrameDistance([double[]]$a, [double[]]$b) {
    $sum = 0.0
    for ($i = 0; $i -lt $a.Count; $i++) {
        $sum += [math]::Pow($a[$i] - $b[$i], 2)
    }
    return $sum
}

function Get-DtwDistance([object[]]$seqA, [object[]]$seqB) {
    $n = $seqA.Count
    $m = $seqB.Count
    if ($n -eq 0 -or $m -eq 0) {
        throw "Cannot compute DTW for empty sequence."
    }

    $prevCost = New-Object double[] ($m + 1)
    $currCost = New-Object double[] ($m + 1)
    $prevLen = New-Object int[] ($m + 1)
    $currLen = New-Object int[] ($m + 1)

    for ($j = 0; $j -le $m; $j++) {
        $prevCost[$j] = [double]::PositiveInfinity
        $currCost[$j] = [double]::PositiveInfinity
        $prevLen[$j] = 0
        $currLen[$j] = 0
    }
    $prevCost[0] = 0.0

    for ($i = 1; $i -le $n; $i++) {
        $currCost[0] = [double]::PositiveInfinity
        $currLen[0] = 0

        for ($j = 1; $j -le $m; $j++) {
            $diag = $prevCost[$j - 1]
            $up = $prevCost[$j]
            $left = $currCost[$j - 1]

            $best = $diag
            $bestLen = $prevLen[$j - 1]

            if ($up -lt $best) {
                $best = $up
                $bestLen = $prevLen[$j]
            }
            if ($left -lt $best) {
                $best = $left
                $bestLen = $currLen[$j - 1]
            }

            $local = Get-FrameDistance $seqA[$i - 1].Features $seqB[$j - 1].Features
            $currCost[$j] = $local + $best
            $currLen[$j] = $bestLen + 1
        }

        $tmpCost = $prevCost
        $prevCost = $currCost
        $currCost = $tmpCost

        $tmpLen = $prevLen
        $prevLen = $currLen
        $currLen = $tmpLen
    }

    return $prevCost[$m] / [math]::Max(1, $prevLen[$m])
}

function Format-Float([double]$value) {
    return $value.ToString("0.000000", [System.Globalization.CultureInfo]::InvariantCulture) + "f"
}

function Get-OutputPath([string]$path) {
    if ([System.IO.Path]::IsPathRooted($path)) {
        return $path
    }
    return [System.IO.Path]::GetFullPath((Join-Path (Get-Location) $path))
}

function Write-LinesNoBom([string]$path, [string[]]$lines) {
    [System.IO.File]::WriteAllLines((Get-OutputPath $path), $lines, $Utf8NoBom)
}

function Write-TextNoBom([string]$path, [string]$text) {
    [System.IO.File]::WriteAllText((Get-OutputPath $path), $text, $Utf8NoBom)
}

function Get-ActionConstName([string]$action) {
    return ($action.ToUpperInvariant() -replace '[^A-Z0-9]+', '_')
}

function Get-DownsampledFrames([object[]]$frames, [int]$targetCount) {
    if ($frames.Count -le $targetCount) {
        return @($frames)
    }

    $out = @()
    for ($i = 0; $i -lt $targetCount; $i++) {
        $idx = [int][math]::Round($i * ($frames.Count - 1) / [math]::Max(1, $targetCount - 1))
        $out += $frames[$idx]
    }
    return $out
}

if (!(Test-Path -LiteralPath $SessionFile)) {
    throw "Session file not found: $SessionFile"
}

$trials = @{}
$accepted = New-Object System.Collections.Generic.HashSet[string]
$rejected = New-Object System.Collections.Generic.HashSet[string]
$currentKey = $null
$flexStraightRaw = $null
$flexBentRaw = $null

foreach ($line in Get-Content -LiteralPath $SessionFile) {
    $trimmed = $line.Trim()
    if ($trimmed.Length -eq 0 -or $trimmed.StartsWith("#")) {
        continue
    }

    $parts = $trimmed -split ","
    $tag = $parts[0].Trim()

    if ($tag -eq "TRIAL_START" -and $parts.Count -ge 3) {
        $action = $parts[1].Trim()
        $trialId = $parts[2].Trim()
        $currentKey = New-TrialKey $action $trialId
        if (!$trials.ContainsKey($currentKey)) {
            $trials[$currentKey] = @{
                Action = $action
                TrialId = $trialId
                Frames = New-Object System.Collections.Generic.List[object]
            }
        }
        continue
    }

    if ($tag -eq "TRIAL_END" -and $parts.Count -ge 3) {
        $currentKey = $null
        continue
    }

    if ($tag -eq "TRIAL_ACCEPT" -and $parts.Count -ge 3) {
        [void]$accepted.Add((New-TrialKey $parts[1].Trim() $parts[2].Trim()))
        continue
    }

    if ($tag -eq "TRIAL_REJECT" -and $parts.Count -ge 3) {
        [void]$rejected.Add((New-TrialKey $parts[1].Trim() $parts[2].Trim()))
        continue
    }

    if ($tag -eq "FLEX_STRAIGHT_END" -and $parts.Count -ge 2) {
        $flexStraightRaw = [int](Parse-Double $parts[1])
        continue
    }

    if ($tag -eq "FLEX_BENT_END" -and $parts.Count -ge 2) {
        $flexBentRaw = [int](Parse-Double $parts[1])
        continue
    }

    if ($tag -eq "DATA" -and $parts.Count -ge 15) {
        $phase = $parts[2].Trim()
        if ($phase -ne "TRIAL") {
            continue
        }

        $trialId = $parts[3].Trim()
        $action = $parts[4].Trim()
        $key = New-TrialKey $action $trialId
        if (!$trials.ContainsKey($key)) {
            $trials[$key] = @{
                Action = $action
                TrialId = $trialId
                Frames = New-Object System.Collections.Generic.List[object]
            }
        }

        if ($parts.Count -ge 23) {
            $upperAngle = Parse-Double $parts[17]
            $forearmAngle = Parse-Double $parts[18]
            $flexRaw = [int](Parse-Double $parts[19])
            $flexAngle = Parse-Double $parts[20]
            $pressureRaw = [int](Parse-Double $parts[21])
            $pressurePressed = [int](Parse-Double $parts[22])

            $upperFeature = if ($UseRelativeAngles) { $upperAngle / 90.0 } else { 0.0 }
            $forearmFeature = if ($UseRelativeAngles) { $forearmAngle / 90.0 } else { 0.0 }

            $features = [double[]]@(
                (Parse-Double $parts[5]),
                (Parse-Double $parts[6]),
                (Parse-Double $parts[7]),
                (Parse-Double $parts[11]),
                (Parse-Double $parts[12]),
                (Parse-Double $parts[13]),
                $upperFeature,
                $forearmFeature,
                ($flexAngle / $FlexScale)
            )
        } else {
            $upperAngle = 0.0
            $forearmAngle = 0.0
            $flexRaw = [int](Parse-Double $parts[11])
            $flexAngle = Parse-Double $parts[12]
            $pressureRaw = [int](Parse-Double $parts[13])
            $pressurePressed = [int](Parse-Double $parts[14])

            $features = [double[]]@(
                (Parse-Double $parts[5]),
                (Parse-Double $parts[6]),
                (Parse-Double $parts[7]),
                (Parse-Double $parts[8]),
                (Parse-Double $parts[9]),
                (Parse-Double $parts[10]),
                ($flexAngle / $FlexScale)
            )
        }

        $trials[$key].Frames.Add([pscustomobject]@{
            TimeMs = [int](Parse-Double $parts[1])
            Features = $features
            UpperAngle = $upperAngle
            ForearmAngle = $forearmAngle
            FlexRaw = $flexRaw
            FlexAngle = $flexAngle
            PressureRaw = $pressureRaw
            PressurePressed = $pressurePressed
        })
    }
}

$acceptedTrials = @()
foreach ($key in $accepted) {
    if ($rejected.Contains($key)) {
        continue
    }
    if ($trials.ContainsKey($key) -and $trials[$key].Frames.Count -gt 1) {
        $acceptedTrials += [pscustomobject]@{
            Key = $key
            Action = $trials[$key].Action
            TrialId = $trials[$key].TrialId
            Frames = [object[]]$trials[$key].Frames
        }
    }
}

if ($acceptedTrials.Count -eq 0) {
    throw "No accepted trials found. Add TRIAL_ACCEPT markers and DATA rows to $SessionFile."
}

$byAction = $acceptedTrials | Group-Object Action
$report = New-Object System.Collections.Generic.List[string]
$report.Add("action,accepted_trials,trial_id,nearest_dtw_distance,dtw_threshold,frames")

$templateActions = @()
$headerTemplates = @()

foreach ($group in $byAction) {
    $actionTrials = @($group.Group)
    foreach ($trial in $actionTrials) {
        $trial | Add-Member -NotePropertyName DtwFrames -NotePropertyValue (Get-DownsampledFrames $trial.Frames $MaxTemplateFrames) -Force
    }
    if ($actionTrials.Count -lt 2) {
        Write-Warning "$($group.Name) has only $($actionTrials.Count) accepted trial. DTW threshold needs at least 2 accepted trials."
        continue
    }

    $nearestDistances = @()
    foreach ($trial in $actionTrials) {
        $nearest = [double]::PositiveInfinity
        foreach ($other in $actionTrials) {
            if ($trial.Key -eq $other.Key) {
                continue
            }
            $distance = Get-DtwDistance $trial.DtwFrames $other.DtwFrames
            if ($distance -lt $nearest) {
                $nearest = $distance
            }
        }
        $nearestDistances += [pscustomobject]@{
            Trial = $trial
            Distance = $nearest
        }
    }

    $maxNearest = ($nearestDistances | Measure-Object -Property Distance -Maximum).Maximum
    $multiplier = if ($actionTrials.Count -le $SmallSampleMax) { $SmallSampleMultiplier } else { $ThresholdMultiplier }
    $threshold = $maxNearest * $multiplier

    foreach ($item in $nearestDistances) {
        $report.Add("$($group.Name),$($actionTrials.Count),$($item.Trial.TrialId),$($item.Distance),$threshold,$($item.Trial.Frames.Count)")
    }

    $selectedTrials = @($nearestDistances |
        Sort-Object Distance |
        Select-Object -First ([math]::Min($MaxTemplatesPerAction, $nearestDistances.Count)) |
        ForEach-Object { $_.Trial })

    $templateActions += [pscustomobject]@{
        action = $group.Name
        acceptedTrials = $actionTrials.Count
        dtwThreshold = $threshold
        maxTemplateFrames = $MaxTemplateFrames
        featureOrder = @("imu1Ax", "imu1Ay", "imu1Az", "imu2Ax", "imu2Ay", "imu2Az", "upperAngleDiv90", "forearmAngleDiv90", "flexAngleDiv$FlexScale")
        templates = @($selectedTrials | ForEach-Object {
            $downsampled = Get-DownsampledFrames $_.Frames $MaxTemplateFrames
            [pscustomobject]@{
                trialId = $_.TrialId
                originalFrames = $_.Frames.Count
                frames = @($downsampled | ForEach-Object { ,@($_.Features) })
            }
        })
    }

    $headerTemplates += [pscustomobject]@{
        Action = $group.Name
        Threshold = $threshold
        Trials = $selectedTrials
    }
}

$reportDir = Split-Path -Parent $ReportOut
$templateDir = Split-Path -Parent $TemplateOut
$headerDir = Split-Path -Parent $HeaderOut
if ($reportDir -and !(Test-Path -LiteralPath $reportDir)) {
    New-Item -ItemType Directory -Path $reportDir | Out-Null
}
if ($templateDir -and !(Test-Path -LiteralPath $templateDir)) {
    New-Item -ItemType Directory -Path $templateDir | Out-Null
}
if ($headerDir -and !(Test-Path -LiteralPath $headerDir)) {
    New-Item -ItemType Directory -Path $headerDir | Out-Null
}

Write-LinesNoBom $ReportOut ([string[]]$report)
Write-TextNoBom $TemplateOut ($templateActions | ConvertTo-Json -Depth 12)

$header = New-Object System.Collections.Generic.List[string]
$header.Add("#ifndef DTW_CALIBRATION_DATA_H")
$header.Add("#define DTW_CALIBRATION_DATA_H")
$header.Add("")
$header.Add("// Generated by tools/compute_dtw_thresholds.ps1")
$header.Add("// Source session: $SessionFile")
$header.Add("")
$header.Add("#define DTW_FEATURE_COUNT 9")
$header.Add("#define DTW_MAX_TEMPLATE_FRAMES $MaxTemplateFrames")
$header.Add("#define DTW_FLEX_STRAIGHT_RAW $(if ($null -ne $flexStraightRaw) { $flexStraightRaw } else { 0 })")
$header.Add("#define DTW_FLEX_BENT_RAW $(if ($null -ne $flexBentRaw) { $flexBentRaw } else { 0 })")
$header.Add("#define DTW_HAS_FLEX_CALIBRATION $(if ($null -ne $flexStraightRaw -and $null -ne $flexBentRaw) { 1 } else { 0 })")
$header.Add("#define DTW_USE_RELATIVE_ANGLES $(if ($UseRelativeAngles) { 1 } else { 0 })")
$header.Add("")
$header.Add("enum DtwActionId {")
$header.Add("    DTW_ACTION_NONE = 0,")
$actionIndex = 1
$knownActions = @("HALF_RAISED", "PICKING")
foreach ($actionGroup in $headerTemplates) {
    $name = Get-ActionConstName $actionGroup.Action
    if ($knownActions -notcontains $name) {
        $knownActions += $name
    }
}
foreach ($name in $knownActions) {
    $header.Add("    DTW_ACTION_$name = $actionIndex,")
    $actionIndex++
}
$header.Add("};")
$header.Add("")
$header.Add("struct DtwTemplateDef {")
$header.Add("    uint8_t actionId;")
$header.Add("    uint8_t frameCount;")
$header.Add("    float threshold;")
$header.Add("    const float *frames;")
$header.Add("};")
$header.Add("")

$templateVarNames = @()
$templateNumber = 0
foreach ($actionGroup in $headerTemplates) {
    $actionConst = "DTW_ACTION_" + (Get-ActionConstName $actionGroup.Action)
    $selected = @($actionGroup.Trials)
    foreach ($trial in $selected) {
        $downsampled = $trial.DtwFrames
        $varName = "DTW_TEMPLATE_${templateNumber}_FRAMES"
        $templateVarNames += [pscustomobject]@{
            VarName = $varName
            ActionConst = $actionConst
            FrameCount = $downsampled.Count
            Threshold = $actionGroup.Threshold
        }
        $header.Add("const float $varName[] = {")
        foreach ($frame in $downsampled) {
            $lineValues = @()
            foreach ($feature in $frame.Features) {
                $lineValues += (Format-Float $feature)
            }
            $header.Add("    " + ($lineValues -join ", ") + ",")
        }
        $header.Add("};")
        $header.Add("")
        $templateNumber++
    }
}

$header.Add("#define DTW_TEMPLATE_COUNT $($templateVarNames.Count)")
$header.Add("const DtwTemplateDef DTW_TEMPLATES[] = {")
foreach ($item in $templateVarNames) {
    $header.Add("    {$($item.ActionConst), $($item.FrameCount), $(Format-Float $item.Threshold), $($item.VarName)},")
}
$header.Add("};")
$header.Add("")
$header.Add("#endif")
Write-LinesNoBom $HeaderOut ([string[]]$header)

Write-Host "Generated $ReportOut"
Write-Host "Generated $TemplateOut"
Write-Host "Generated $HeaderOut"
Write-Host ("Accepted trials: " + (($acceptedTrials | ForEach-Object { $_.Key }) -join ", "))
if ($null -ne $flexStraightRaw -and $null -ne $flexBentRaw) {
    Write-Host "Flex calibration: straight=$flexStraightRaw, bent=$flexBentRaw"
} else {
    Write-Warning "Flex calibration markers not found. Formal sketch will still require f/b manual calibration."
}
