rem Re-construct CMake-friendly structure (dir with include and lib subdirs and both debug and release libs in lib)
rem This is required for the OpenSceneGraph-releated projects autodetect/find scripts to work
set PACKAGEDIR=%1
set OUTDIR=%2
rem rmdir /s /q %OUTDIR%
rem mkdir %OUTDIR%
robocopy /s /NFL /NDL %PACKAGEDIR%\build\native\include %OUTDIR%\include
rem mkdir %OUTDIR%\lib
robocopy /NFL %PACKAGEDIR%\build\native\lib\x64\v120\Debug\Desktop %OUTDIR%\lib\
robocopy /NFL %PACKAGEDIR%\build\native\lib\x64\v120\Release\Desktop %OUTDIR%\lib\
robocopy /NFL %PACKAGEDIR%\build\native\lib\x64\v120\Debug\dynamic %OUTDIR%\lib\
robocopy /NFL %PACKAGEDIR%\build\native\lib\x64\v120\Release\dynamic %OUTDIR%\lib\
