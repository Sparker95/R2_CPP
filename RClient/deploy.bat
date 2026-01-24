C:
call C:\Qt\6.10.1\mingw_64\bin\qtenv2.bat

set BUILD_DIR="D:\Projects\Robot2\CPP\RClient\build\Desktop_Qt_6_10_1_MinGW_64_bit-Release"

set OUT_ARCHIVE="D:\Projects\Robot2\CPP\RClient\build\RClient.zip"

windeployqt.exe --release %BUILD_DIR%

echo Archiving
set PATH="C:\Program Files\7-Zip";%PATH%

del /f %OUT_ARCHIVE%

7z.exe a %OUT_ARCHIVE% %BUILD_DIR%

pause