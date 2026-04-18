@echo off
set PATH=C:\dev\vcpkg_export\installed\x64-windows-release\bin;C:\dev\rtabmap\build\bin;%PATH%
set QT_PLUGIN_PATH=C:\dev\vcpkg_export\installed\x64-windows-release\Qt6\plugins
start "" "C:\dev\rtabmap\build\bin\RTABMap.exe"
