$name = $args[0]
(Get-Content .cproject) | ForEach-Object { $_ -replace "HID_Gamepad_example", $name } | Set-Content .cproject
(Get-Content .project) | ForEach-Object { $_ -replace "HID_Gamepad_example", $name } | Set-Content .project
(Get-Content Link.ld) | ForEach-Object { $_ -replace "HID_Gamepad_example", $name } | Set-Content Link.ld
Write-Output "Job is done"