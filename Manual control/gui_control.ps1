<#
.SYNOPSIS
    Realtime GUI Controller for ESP8266 4-DOF Arm (MG90S Servos: GPIO 1 = Elbow Pitch J3, GPIO 3 = Shoulder Pitch J2, GPIO 5 = Base Yaw J1, GPIO 4 = Wrist Pitch J4).
.DESCRIPTION
    Provides interactive trackbar sliders to control each servo angle in real time via UDP.
#>

Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing

# --- UDP Sender Functionality ---
$global:udpClient = New-Object System.Net.Sockets.UdpClient
$global:udpClient.EnableBroadcast = $true
$global:port = 8888
$deg = [char]176

function Send-UDP {
    param([int]$a1, [int]$a2, [int]$a3, [int]$a4)
    
    $ip = $txtIP.Text.Trim()
    if ([string]::IsNullOrWhiteSpace($ip)) { $ip = "255.255.255.255" }

    try {
        $payload = [System.Text.Encoding]::ASCII.GetBytes("$a1,$a2,$a3,$a4")
        $global:udpClient.Send($payload, $payload.Length, $ip, $global:port) | Out-Null
        $lblStatus.Text = "Sent -> IP: $ip`:$global:port | S1: ${a1}${deg} | S2: ${a2}${deg} | S3: ${a3}${deg} | S4: ${a4}${deg}"
        $lblStatus.ForeColor = [System.Drawing.Color]::LimeGreen
    }
    catch {
        $lblStatus.Text = "Error: $_"
        $lblStatus.ForeColor = [System.Drawing.Color]::Crimson
    }
}

# --- Main Form Setup ---
$form = New-Object System.Windows.Forms.Form
$form.Text = "ESP8266 4-DOF Servo Arm Controller"
$form.Size = New-Object System.Drawing.Size(530, 610)
$form.StartPosition = "CenterScreen"
$form.BackColor = [System.Drawing.Color]::FromArgb(24, 24, 37)
$form.ForeColor = [System.Drawing.Color]::FromArgb(205, 214, 244)
$form.FormBorderStyle = "FixedSingle"
$form.MaximizeBox = $false
$form.Font = New-Object System.Drawing.Font("Segoe UI", 10)

# Header Title
$lblTitle = New-Object System.Windows.Forms.Label
$lblTitle.Text = "Realtime 4-DOF Servo Control"
$lblTitle.Font = New-Object System.Drawing.Font("Segoe UI", 14, [System.Drawing.FontStyle]::Bold)
$lblTitle.ForeColor = [System.Drawing.Color]::FromArgb(137, 180, 250)
$lblTitle.Location = New-Object System.Drawing.Point(20, 15)
$lblTitle.Size = New-Object System.Drawing.Size(350, 30)
$form.Controls.Add($lblTitle)

# Target IP Input
$lblIP = New-Object System.Windows.Forms.Label
$lblIP.Text = "Target IP:"
$lblIP.Location = New-Object System.Drawing.Point(20, 55)
$lblIP.Size = New-Object System.Drawing.Size(75, 25)
$form.Controls.Add($lblIP)

$txtIP = New-Object System.Windows.Forms.TextBox
$txtIP.Text = "192.168.137.78"
$txtIP.Location = New-Object System.Drawing.Point(95, 52)
$txtIP.Size = New-Object System.Drawing.Size(160, 28)
$txtIP.BackColor = [System.Drawing.Color]::FromArgb(49, 50, 68)
$txtIP.ForeColor = [System.Drawing.Color]::White
$txtIP.BorderStyle = "FixedSingle"
$form.Controls.Add($txtIP)

$lblPort = New-Object System.Windows.Forms.Label
$lblPort.Text = "Port: 8888"
$lblPort.ForeColor = [System.Drawing.Color]::FromArgb(166, 173, 200)
$lblPort.Location = New-Object System.Drawing.Point(270, 55)
$lblPort.Size = New-Object System.Drawing.Size(100, 25)
$form.Controls.Add($lblPort)

# --- Helper Function for Slider Row ---
function Create-SliderGroup {
    param(
        [string]$Name,
        [string]$PinLabel,
        [int]$YPos
    )

    $lbl = New-Object System.Windows.Forms.Label
    $lbl.Text = "$Name ($PinLabel): 90${deg}"
    $lbl.Font = New-Object System.Drawing.Font("Segoe UI", 10, [System.Drawing.FontStyle]::Bold)
    $lbl.Location = New-Object System.Drawing.Point(20, $YPos)
    $lbl.Size = New-Object System.Drawing.Size(470, 25)
    $form.Controls.Add($lbl)

    $track = New-Object System.Windows.Forms.TrackBar
    $track.Minimum = 0
    $track.Maximum = 180
    $track.Value = 90
    $track.TickFrequency = 15
    $track.Location = New-Object System.Drawing.Point(15, ($YPos + 25))
    $track.Size = New-Object System.Drawing.Size(480, 45)
    $form.Controls.Add($track)

    return @{ Label = $lbl; Track = $track }
}

$group1 = Create-SliderGroup -Name "Servo 1" -PinLabel "GPIO 1 / Elbow Pitch (J3)" -YPos 95
$group2 = Create-SliderGroup -Name "Servo 2" -PinLabel "GPIO 3 / Shoulder Pitch (J2)" -YPos 170
$group3 = Create-SliderGroup -Name "Servo 3" -PinLabel "GPIO 5 / Base Yaw (J1)" -YPos 245
$group4 = Create-SliderGroup -Name "Servo 4" -PinLabel "GPIO 4 / Wrist Pitch (J4)" -YPos 320

# Event handler for slider changes
$updateAngles = {
    $a1 = $group1.Track.Value
    $a2 = $group2.Track.Value
    $a3 = $group3.Track.Value
    $a4 = $group4.Track.Value

    $group1.Label.Text = "Servo 1 (GPIO 1 / Elbow Pitch J3): ${a1}${deg}"
    $group2.Label.Text = "Servo 2 (GPIO 3 / Shoulder Pitch J2): ${a2}${deg}"
    $group3.Label.Text = "Servo 3 (GPIO 5 / Base Yaw J1): ${a3}${deg}"
    $group4.Label.Text = "Servo 4 (GPIO 4 / Wrist Pitch J4): ${a4}${deg}"

    Send-UDP -a1 $a1 -a2 $a2 -a3 $a3 -a4 $a4
}

$group1.Track.Add_Scroll($updateAngles)
$group2.Track.Add_Scroll($updateAngles)
$group3.Track.Add_Scroll($updateAngles)
$group4.Track.Add_Scroll($updateAngles)

# --- Preset Buttons ---
$panelPresets = New-Object System.Windows.Forms.Panel
$panelPresets.Location = New-Object System.Drawing.Point(20, 405)
$panelPresets.Size = New-Object System.Drawing.Size(480, 45)
$form.Controls.Add($panelPresets)

function Create-Button {
    param([string]$Text, [int]$X, [int]$Width = 108, [scriptblock]$Action)
    $btn = New-Object System.Windows.Forms.Button
    $btn.Text = $Text
    $btn.Location = New-Object System.Drawing.Point($X, 0)
    $btn.Size = New-Object System.Drawing.Size($Width, 36)
    $btn.FlatStyle = "Flat"
    $btn.FlatAppearance.BorderSize = 1
    $btn.FlatAppearance.BorderColor = [System.Drawing.Color]::FromArgb(137, 180, 250)
    $btn.BackColor = [System.Drawing.Color]::FromArgb(49, 50, 68)
    $btn.ForeColor = [System.Drawing.Color]::White
    $btn.Font = New-Object System.Drawing.Font("Segoe UI", 9)
    $btn.Add_Click($Action)
    $panelPresets.Controls.Add($btn)
}

Create-Button -Text "0${deg} (Min)" -X 0 -Width 108 -Action {
    $group1.Track.Value = 0; $group2.Track.Value = 0; $group3.Track.Value = 0; $group4.Track.Value = 0
    &$updateAngles
}

Create-Button -Text "90${deg} (Neutral)" -X 118 -Width 118 -Action {
    $group1.Track.Value = 90; $group2.Track.Value = 90; $group3.Track.Value = 90; $group4.Track.Value = 90
    &$updateAngles
}

Create-Button -Text "180${deg} (Max)" -X 246 -Width 108 -Action {
    $group1.Track.Value = 180; $group2.Track.Value = 180; $group3.Track.Value = 180; $group4.Track.Value = 180
    &$updateAngles
}

# Auto-Sweep Toggle Button
$script:isSweeping = $false
$timer = New-Object System.Windows.Forms.Timer
$timer.Interval = 30
$script:sweepAngle = 0
$script:sweepDir = 5

$timer.Add_Tick({
    $script:sweepAngle += $script:sweepDir
    if ($script:sweepAngle -ge 180) { $script:sweepAngle = 180; $script:sweepDir = -5 }
    if ($script:sweepAngle -le 0) { $script:sweepAngle = 0; $script:sweepDir = 5 }

    $group1.Track.Value = $script:sweepAngle
    $group2.Track.Value = $script:sweepAngle
    $group3.Track.Value = $script:sweepAngle
    $group4.Track.Value = $script:sweepAngle
    &$updateAngles
})

Create-Button -Text "Sweep Test" -X 364 -Width 108 -Action {
    if ($script:isSweeping) {
        $timer.Stop()
        $script:isSweeping = $false
        $this.Text = "Sweep Test"
        $this.BackColor = [System.Drawing.Color]::FromArgb(49, 50, 68)
    } else {
        $timer.Start()
        $script:isSweeping = $true
        $this.Text = "Stop Sweep"
        $this.BackColor = [System.Drawing.Color]::FromArgb(243, 139, 168)
    }
}

# --- Status Bar ---
$lblStatus = New-Object System.Windows.Forms.Label
$lblStatus.Text = "Ready. Drag sliders to control 4 servos."
$lblStatus.Font = New-Object System.Drawing.Font("Segoe UI", 9.5)
$lblStatus.ForeColor = [System.Drawing.Color]::FromArgb(166, 173, 200)
$lblStatus.Location = New-Object System.Drawing.Point(20, 475)
$lblStatus.Size = New-Object System.Drawing.Size(470, 40)
$form.Controls.Add($lblStatus)

# On Form Closing - Clean up socket
$form.Add_FormClosing({
    $timer.Stop()
    if ($global:udpClient) { $global:udpClient.Close() }
})

# Show GUI
$form.ShowDialog() | Out-Null
