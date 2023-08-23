@echo off
echo CH32V203C8T6_USB_VCP: new project for RVMSIS	
cd C:\RVMSIS\CH32V203C8T6_USB_VCP
set /p name="Enter project name:"
md %name%
xcopy /y /o /e /d "CH32V203C8T6_USB_VCP" %name%
cd %name%
Powershell.exe -executionpolicy remotesigned -File "C:\RVMSIS\CH32V203C8T6_USB_VCP\script.ps1" %name%
explorer.exe C:\RVMSIS\CH32V203C8T6_USB_VCP