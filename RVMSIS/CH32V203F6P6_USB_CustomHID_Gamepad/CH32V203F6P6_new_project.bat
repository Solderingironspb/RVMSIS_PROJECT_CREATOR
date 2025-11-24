@echo off
net session >nul 2>&1
if %errorLevel% neq 0 (
    echo Running a script as an administrator...
    PowerShell -Command "Start-Process cmd -ArgumentList '/c %0' -Verb RunAs"
    exit /b
)

echo CH32V203F6P6_USB_Custom_HID_Gamepad: new project for RVMSIS	
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"
set /p name="Enter project name:"
md %name%
xcopy /y /o /e /d "Clean_project" %name%
cd %name%
Powershell.exe -executionpolicy bypass -File "%SCRIPT_DIR%\script.ps1" %name%
explorer.exe "%SCRIPT_DIR%"