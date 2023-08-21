@echo off
echo CH32V203C8T6_USB_CustomHID_Gamepad: new project for RVMSIS	
cd C:\RVMSIS\CH32V203C8T6_USB_CustomHID_Gamepad
set /p name="Enter project name:"
md %name%
xcopy /y /o /e /d "HID_Gamepad_example" %name%
cd %name%
Powershell.exe -executionpolicy remotesigned -File "C:\RVMSIS\CH32V203C8T6_USB_CustomHID_Gamepad\script.ps1" %name%
explorer.exe C:\RVMSIS\CH32V203C8T6_USB_CustomHID_Gamepad