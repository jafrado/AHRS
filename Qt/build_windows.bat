@echo off
@echo Setting up Visual Studio Dev Environment
@call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat"
@echo Running Cmake
rd /s /q build
mkdir build
cd build
cmake .. -DBoost_CHRONO_LIBRARY_RELEASE:FILEPATH="C:/vcpkg/installed/x64-windows/lib/boost_atomic-vc140-mt.lib" -DBoost_FILESYSTEM_LIBRARY_RELEASE:FILEPATH="C:/vcpkg/installed/x64-windows/lib/boost_filesystem-vc140-mt.lib" -DPCL_DIR:PATH="C:/vcpkg/packages/pcl_x64-windows/share/pcl" -DQt5_Dir:PATH="C:/vcpkg/packages/qt5-base_x64-windows/share/cmake/Qt5" -DBoost_CHRONO_LIBRARY_DEBUG:FILEPATH="C:/vcpkg/installed/x64-windows/debug/lib/boost_chrono-vc140-mt-gd.lib" -DBoost_SYSTEM_LIBRARY_DEBUG:FILEPATH="C:/vcpkg/installed/x64-windows/debug/lib/boost_system-vc140-mt-gd.lib" -DBoost_THREAD_LIBRARY_RELEASE:FILEPATH="C:/vcpkg/installed/x64-windows/lib/boost_thread-vc140-mt.lib" -DBoost_DATE_TIME_LIBRARY_DEBUG:FILEPATH="C:/vcpkg/installed/x64-windows/debug/lib/boost_date_time-vc140-mt-gd.lib" -DEIGEN_INCLUDE_DIR:PATH="C:/vcpkg/packages/eigen3_x64-windows/include/eigen3" -DBoost_IOSTREAMS_LIBRARY_RELEASE:FILEPATH="C:/vcpkg/installed/x64-windows/lib/boost_iostreams-vc140-mt.lib" -Dlz4_DIR:PATH="C:/vcpkg/packages/lz4_x64-windows/share/lz4" -DFLANN_DIR:PATH="C:/vcpkg/packages/flann_x64-windows/share/flann" -DBoost_FILESYSTEM_LIBRARY_DEBUG:FILEPATH="C:/vcpkg/installed/x64-windows/debug/lib/boost_filesystem-vc140-mt-gd.lib" -DQt5_DIR:PATH="C:/vcpkg/packages/qt5-base_x64-windows/share/cmake/Qt5" -DBoost_IOSTREAMS_LIBRARY_DEBUG:FILEPATH="C:/vcpkg/installed/x64-windows/debug/lib/boost_iostreams-vc140-mt-gd.lib" -DBoost_DATE_TIME_LIBRARY_RELEASE:FILEPATH="C:/vcpkg/installed/x64-windows/lib/boost_date_time-vc140-mt.lib" -DBoost_INCLUDE_DIR:PATH="C:/vcpkg/installed/x64-windows/include" -DBoost_THREAD_LIBRARY_DEBUG:FILEPATH="C:/vcpkg/installed/x64-windows/debug/lib/boost_thread-vc140-mt-gd.lib" -DBoost_SYSTEM_LIBRARY_RELEASE:FILEPATH="C:/vcpkg/installed/x64-windows/lib/boost_system-vc140-mt.lib" -DVTK_DIR:PATH="C:/vcpkg/packages/vtk_x64-windows/share/vtk"  -DQt5Charts_DIR:PATH="C:/vcpkg/packages/qt5-charts_x64-windows/share/cmake/Qt5Charts"  -DCMAKE_EXE_LINKER_FLAGS_RELEASE:STRING="/INCREMENTAL:NO /SUBSYSTEM:WINDOWS /ENTRY:mainCRTStartup"


@echo Starting build [Debug]
"C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\IDE\devenv.com" /rebuild Debug kiwi-a1.sln

@echo Starting build [Release]
"C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\IDE\devenv.com" /rebuild Release kiwi-a1.sln

@REM install all DLL's from using dumpbin /dependents (and devenv from command line)

@echo Copying DLL's for Debug Build ...
xcopy /I /E /s C:\vcpkg\installed\x64-windows\debug\plugins\platforms Debug\platforms\
xcopy /I /E /s ..\icons Debug\icons\
mkdir Debug\config
mkdir Debug\config\grey
mkdir Debug\config\ibpd
mkdir Debug\config\kiwid
mkdir Debug\calibration
mkdir Release\calibration\grey
mkdir Release\calibration\ibpd
mkdir Release\calibration\kiwid
xcopy /I /E /s ..\demo-configs\grey Debug\config\grey\
xcopy /I /E /s ..\demo-configs\kiwid Debug\config\kiwid
xcopy /I /E /s ..\configs\ibpd-*.* Debug\config\ibpd\
xcopy /I /E /s ..\calibration\grey Debug\calibration\grey
xcopy /I /E /s ..\calibration\ibpd Debug\calibration\ibpd
xcopy /I /E /s ..\calibration\kiwid Debug\calibration\kiwid
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\boost_date_time-vc142-mt-gd-x64-1_71.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\boost_filesystem-vc142-mt-gd-x64-1_71.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\boost_iostreamsd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\boost_thread-vc142-mt-gd-x64-1_71.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\bz2d.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\jpeg62.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\libpng16.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\libpng16d.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\lz4d.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\lzmad.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\packet.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\pcl_common_debug.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\pcl_io_debug.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\pcl_io_ply_debug.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\tiffd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkChartsCore-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkCommonColor-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkCommonComputationalGeometry-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkFiltersCore-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkFiltersExtraction-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkFiltersGeneral-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkFiltersGeometry-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkFiltersHybrid-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkFiltersModeling-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkFiltersSources-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkFiltersStatistics-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkgl2ps-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkGUISupportQt-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkImagingColor-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkImagingFourier-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkImagingGeneral-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkImagingHybrid-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkImagingSources-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkInfovisCore-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkInteractionStyle-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkInteractionWidgets-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkRenderingAnnotation-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkRenderingContext2D-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkRenderingContextOpenGL2-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkRenderingCore-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkRenderingFreeType-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkRenderingGL2PSOpenGL2-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkRenderingLOD-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkRenderingOpenGL2-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkViewsContext2D-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkViewsCore-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkCommonCore-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkCommonDataModel-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkCommonExecutionModel-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkCommonMath-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkCommonMisc-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkCommonSystem-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkCommonTransforms-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkDICOMParser-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkImagingCore-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkIOCore-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkIOGeometry-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkIOImage-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkIOLegacy-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkIOPLY-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtkmetaio-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\vtksys-8.2.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\wpcap.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\zlib1.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\zlibd1.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\zstdd.dll Debug
copy /b/v/y C:\Windows\System32\VCRUNTIME140d.dll Debug
copy /b/v/y C:\Windows\System32\kernel32.dll Debug
copy /b/v/y C:\Windows\System32\ucrtbased.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\freetyped.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\glew32d.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\harfbuzz.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\libeay32.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\pcl_kdtree_debug.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\pcl_visualization_debug.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\pcre2-16d.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\Qt5Cored.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\Qt5Guid.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\Qt5Networkd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\Qt5Qmld.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\Qt5QuickControls2d.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\Qt5Quickd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\Qt5QuickShapesd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\Qt5QuickTemplates2d.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\Qt5Svgd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\Qt5VirtualKeyboardd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\Qt5Widgetsd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\Qt5Chartsd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\ssleay32.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\tiffd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\webpd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\bin\webpdemuxd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\plugins\qmltooling\qmldbg_debuggerd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\plugins\qmltooling\qmldbg_inspectord.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\plugins\qmltooling\qmldbg_locald.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\plugins\qmltooling\qmldbg_messagesd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\plugins\qmltooling\qmldbg_natived.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\plugins\qmltooling\qmldbg_nativedebuggerd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\plugins\qmltooling\qmldbg_previewd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\plugins\qmltooling\qmldbg_profilerd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\plugins\qmltooling\qmldbg_quickprofilerd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\plugins\qmltooling\qmldbg_serverd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\plugins\qmltooling\qmldbg_tcpd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\plugins\bearer\qgenericbearerd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\lz4.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\plugins\imageformats\qgifd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\plugins\imageformats\qicnsd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\plugins\imageformats\qicod.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\plugins\imageformats\qjpegd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\plugins\imageformats\qwbmpd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\plugins\imageformats\qwebpd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\plugins\imageformats\qsvgd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\plugins\imageformats\qtgad.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\plugins\imageformats\qtiffd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\plugins\scenegraph\qsgd3d12backendd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\plugins\iconengines\qsvgicond.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\plugins\platforminputcontexts\qtvirtualkeyboardplugind.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\plugins\platforms\qwindowsd.dll Debug
copy /b/v/y C:\vcpkg\installed\x64-windows\debug\plugins\styles\qwindowsvistastyled.dll Debug
copy /b/v/y C:\Windows\System32\MSVCP140D.dll Debug
copy /b/v/y C:\Windows\System32\VCRUNTIME140_1D.dll Debug
copy /b/v/y C:\Windows\System32\ADVAPI32.dll Debug

@echo Copying DLL's for Release ...
xcopy /I /E /s C:\vcpkg\installed\x64-windows\plugins\platforms Release\platforms\
xcopy /I /E /s ..\icons Release\icons\
mkdir Release\config
mkdir Release\config\grey
mkdir Release\config\ibpd
mkdir Release\config\kiwid
mkdir Release\calibration\grey
mkdir Release\calibration\ibpd
mkdir Release\calibration\kiwid
xcopy /I /E /s ..\demo-configs\grey Release\config\grey\
xcopy /I /E /s ..\demo-configs\kiwid Release\config\kiwid\
xcopy /I /E /s ..\configs\ibpd-*.* Release\config\ibpd\
xcopy /I /E /s ..\calibration\grey Release\calibration\grey
xcopy /I /E /s ..\calibration\ibpd Release\calibration\ibpd
xcopy /I /E /s ..\calibration\kiwid Release\calibration\kiwid
copy /b/v/y C:\Windows\System32\ADVAPI32.dll Release
copy /b/v/y "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\IDE\api-ms-win-crt-stdio-l1-1-0.dll" Release
copy /b/v/y "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\IDE\api-ms-win-crt-heap-l1-1-0.dll" Release
copy /b/v/y "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\IDE\api-ms-win-crt-runtime-l1-1-0.dll" Release
copy /b/v/y "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\IDE\api-ms-win-crt-convert-l1-1-0.dll" Release
copy /b/v/y "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\IDE\api-ms-win-crt-math-l1-1-0.dll" Release
copy /b/v/y "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\IDE\api-ms-win-crt-locale-l1-1-0.dll" Release
copy /b/v/y "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\IDE\api-ms-win-crt-string-l1-1-0.dll" Release
copy /b/v/y "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\IDE\api-ms-win-crt-time-l1-1-0.dll" Release
copy /b/v/y "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\IDE\api-ms-win-crt-utility-l1-1-0.dll" Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\boost_date_time-vc142-mt-x64-1_71.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\boost_filesystem-vc142-mt-x64-1_71.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\boost_iostreams.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\boost_thread-vc142-mt-x64-1_71.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\bz2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkCommonCore-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkCommonDataModel-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkCommonExecutionModel-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkCommonMath-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkCommonMisc-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkCommonSystem-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkCommonTransforms-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkDICOMParser-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkImagingCore-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkIOCore-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkIOGeometry-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkIOImage-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkIOLegacy-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkIOPLY-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkmetaio-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtksys-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\wpcap.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\zlib1.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\zstd.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkChartsCore-8.2.dll  Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkCommonColor-8.2.dll  Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkCommonComputationalGeometry-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkFiltersCore-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkFiltersExtraction-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkFiltersGeneral-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkFiltersGeometry-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkFiltersHybrid-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkFiltersModeling-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkFiltersSources-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkFiltersStatistics-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkgl2ps-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkGUISupportQt-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkImagingColor-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkImagingFourier-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkImagingGeneral-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkImagingHybrid-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkImagingSources-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkInfovisCore-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkInteractionStyle-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkInteractionWidgets-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkRenderingAnnotation-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkRenderingContext2D-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkRenderingContextOpenGL2-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkRenderingCore-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkRenderingFreeType-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkRenderingGL2PSOpenGL2-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkRenderingLOD-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkRenderingOpenGL2-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkViewsContext2D-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\vtkViewsCore-8.2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\freetype.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\glew32.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\harfbuzz.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\jpeg62.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\libeay32.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\libpng16.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\lz4.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\lzma.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\packet.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\pcl_common_release.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\pcl_io_ply_release.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\pcl_io_release.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\pcl_kdtree_release.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\pcl_visualization_release.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\pcre2-16.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\plugins\bearer\qgenericbearer.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\plugins\imageformats\qgif.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\plugins\imageformats\qicns.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\plugins\imageformats\qico.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\plugins\imageformats\qjpeg.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\plugins\imageformats\qwbmp.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\plugins\imageformats\qwebp.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\plugins\imageformats\qsvg.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\plugins\imageformats\qtga.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\plugins\imageformats\qtiff.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\plugins\qmltooling\qmldbg_debugger.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\plugins\qmltooling\qmldbg_inspector.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\plugins\qmltooling\qmldbg_local.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\plugins\qmltooling\qmldbg_messages.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\plugins\qmltooling\qmldbg_native.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\plugins\qmltooling\qmldbg_nativedebugger.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\plugins\qmltooling\qmldbg_preview.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\plugins\qmltooling\qmldbg_profiler.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\plugins\qmltooling\qmldbg_quickprofiler.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\plugins\qmltooling\qmldbg_server.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\plugins\qmltooling\qmldbg_tcp.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\plugins\scenegraph\qsgd3d12backend.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\plugins\iconengines\qsvgicon.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\plugins\platforminputcontexts\qtvirtualkeyboardplugin.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\webpdemux.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\plugins\platforms\qwindows.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\plugins\styles\qwindowsvistastyle.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\ssleay32.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\tiff.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\webp.dll Release
copy /b/v/y C:\Windows\System32\VCRUNTIME140.dll Release
copy /b/v/y C:\Windows\System32\kernel32.dll Release
copy /b/v/y C:\Windows\System32\VCRUNTIME140_1.dll Release
copy /b/v/y C:\Windows\System32\MSVCP140.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\Qt5Core.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\Qt5Gui.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\Qt5Network.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\Qt5Qml.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\Qt5Quick.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\Qt5QuickControls2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\Qt5QuickShapes.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\Qt5QuickTemplates2.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\Qt5Svg.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\Qt5VirtualKeyboard.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\Qt5Widgets.dll Release
copy /b/v/y C:\vcpkg\installed\x64-windows\bin\Qt5Charts.dll Release


@echo Windows Build Complete

@echo Packaging Release and Debug builds

@SETLOCAL ENABLEDELAYEDEXPANSION
@REM Use WMIC to retrieve date and time
@echo off
FOR /F "skip=1 tokens=1-6" %%A IN ('WMIC Path Win32_LocalTime Get Day^,Hour^,Minute^,Month^,Second^,Year /Format:table') DO (
    IF NOT "%%~F"=="" (
        SET /A SortDate = 10000 * %%F + 100 * %%D + %%A
        set YEAR=!SortDate:~0,4!
        set MON=!SortDate:~4,2!
        set DAY=!SortDate:~6,2!
        @REM Add 1000000 so as to force a prepended 0 if hours less than 10
        SET /A SortTime = 1000000 + 10000 * %%B + 100 * %%C + %%E
        set HOUR=!SortTime:~1,2!
        set MIN=!SortTime:~3,2!
        set SEC=!SortTime:~5,2!
    )
)
@echo on
@set DATECODE=!YEAR!!MON!!DAY!-!HOUR!!MIN!
@echo OURS-Viewer-Windows-x64-%DATECODE%.zip

c:\vcpkg\downloads\tools\7zip-18.1.0-windows\7-Zip.CommandLine.18.1.0\tools\x64\7za -mx9 a OURS-Viewer-Windows-x64-%DATECODE%-Debug.zip Debug
c:\vcpkg\downloads\tools\7zip-18.1.0-windows\7-Zip.CommandLine.18.1.0\tools\x64\7za -mx9 a OURS-Viewer-Windows-x64-%DATECODE%-Release.zip Release


@echo Completed
cd ..

