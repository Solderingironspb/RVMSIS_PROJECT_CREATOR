@echo off
echo CH32V203F6P6_USB_HID: new project for RVMSIS	
cd C:\RVMSIS\CH32V203F6P6_USB_CustomHID_Gamepad
set /p name="Enter project name:"
md %name%
xcopy /y /o /e /d "Clean_project" %name%
cd %name%
Powershell.exe -executionpolicy remotesigned -File "C:\RVMSIS\CH32V203F6P6_USB_CustomHID_Gamepad\script.ps1" %name%
explorer.exe C:\RVMSIS\CH32V203F6P6_USB_CustomHID_Gamepad