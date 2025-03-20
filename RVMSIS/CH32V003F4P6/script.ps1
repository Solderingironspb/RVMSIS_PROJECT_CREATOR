$name = $args[0]
(Get-Content .cproject) | ForEach-Object { $_ -replace "Clean_project", $name } | Set-Content .cproject
(Get-Content .project) | ForEach-Object { $_ -replace "Clean_project", $name } | Set-Content .project
(Get-Content Link.ld) | ForEach-Object { $_ -replace "Clean_project", $name } | Set-Content Link.ld
(Get-Content Makefile) | ForEach-Object { $_ -replace "Clean_project", $name } | Set-Content Makefile
echo "Job is done"