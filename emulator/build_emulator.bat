@echo off
@echo Setting up Visual Studio Dev Environment
@call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat"

@echo Running CMake for AHRSEmulator
rd /s /q build
mkdir build
cd build

cmake .. -DQt5_DIR:PATH="C:/vcpkg/packages/qt5-base_x64-windows/share/cmake/Qt5" ^
         -DQt5SerialPort_DIR:PATH="C:/vcpkg/packages/qt5-serialport_x64-windows/share/cmake/Qt5SerialPort"

@echo Building [Debug]
msbuild AHRSEmulator.sln /p:Configuration=Debug /p:Platform=x64 /m

@echo Building [Release]
msbuild AHRSEmulator.sln /p:Configuration=Release /p:Platform=x64 /m

@echo Copying Qt DLLs for Debug ...
xcopy /I /E /s C:\vcpkg\installed\x64-windows\debug\plugins\platforms Debug\platforms\
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\Qt5Cored.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\Qt5Guid.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\Qt5Widgetsd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\Qt5SerialPortd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\pcre2-16d.dll Debug

@echo Copying Qt DLLs for Release ...
xcopy /I /E /s C:\vcpkg\installed\x64-windows\plugins\platforms Release\platforms\
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\Qt5Core.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\Qt5Gui.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\Qt5Widgets.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\Qt5SerialPort.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\pcre2-16.dll Release

@echo AHRSEmulator build complete.
cd ..
