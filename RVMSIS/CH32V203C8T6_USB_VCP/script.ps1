$name = $args[0]
(Get-Content .cproject) | ForEach-Object { $_ -replace "CH32V203C8T6_USB_VCP", $name } | Set-Content .cproject
(Get-Content .project) | ForEach-Object { $_ -replace "CH32V203C8T6_USB_VCP", $name } | Set-Content .project
(Get-Content Link.ld) | ForEach-Object { $_ -replace "CH32V203C8T6_USB_VCP", $name } | Set-Content Link.ld
echo "Job is done"