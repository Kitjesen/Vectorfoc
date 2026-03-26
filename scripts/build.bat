@echo off
REM VectorFOC Build Script for Windows
REM Usage: build.bat [app|boot|all|clean]

setlocal enabledelayedexpansion

set SCRIPT_DIR=%~dp0
set PROJECT_DIR=%SCRIPT_DIR%..
set TOOLCHAIN_FILE=%PROJECT_DIR%\cmake\gcc-arm-none-eabi.cmake

if "%1"=="" set TARGET=all
if not "%1"=="" set TARGET=%1

if "%TARGET%"=="app" goto build_app
if "%TARGET%"=="boot" goto build_boot
if "%TARGET%"=="all" goto build_all
if "%TARGET%"=="clean" goto clean_all
goto usage

:build_app
echo ==========================================
echo Building Application...
echo ==========================================

if not exist "%PROJECT_DIR%\build" mkdir "%PROJECT_DIR%\build"
cd /d "%PROJECT_DIR%\build"

cmake -G "MinGW Makefiles" -DCMAKE_TOOLCHAIN_FILE="%TOOLCHAIN_FILE%" -DCMAKE_BUILD_TYPE=Release "%PROJECT_DIR%"
if errorlevel 1 goto error

mingw32-make -j4
if errorlevel 1 goto error

echo.
echo Application built: build\VectorFoc.bin
goto end

:build_boot
echo ==========================================
echo Building Bootloader...
echo ==========================================

if not exist "%PROJECT_DIR%\build_boot" mkdir "%PROJECT_DIR%\build_boot"
cd /d "%PROJECT_DIR%\build_boot"

REM Use bootloader CMakeLists
copy /Y "%PROJECT_DIR%\CMakeLists_Bootloader.txt" "%PROJECT_DIR%\CMakeLists.txt.bak" >nul

cmake -G "MinGW Makefiles" -DCMAKE_TOOLCHAIN_FILE="%TOOLCHAIN_FILE%" -DCMAKE_BUILD_TYPE=Release -DBOOTLOADER_BUILD=ON "%PROJECT_DIR%"
if errorlevel 1 goto error

mingw32-make -j4
if errorlevel 1 goto error

echo.
echo Bootloader built: build_boot\VectorFoc_Bootloader.bin
goto end

:build_all
call :build_boot
call :build_app
goto end

:clean_all
echo Cleaning build directories...
if exist "%PROJECT_DIR%\build" rmdir /s /q "%PROJECT_DIR%\build"
if exist "%PROJECT_DIR%\build_boot" rmdir /s /q "%PROJECT_DIR%\build_boot"
echo Done.
goto end

:usage
echo Usage: %0 [app^|boot^|all^|clean]
echo   app   - Build application only
echo   boot  - Build bootloader only
echo   all   - Build both (default)
echo   clean - Clean build directories
goto end

:error
echo.
echo Build failed!
exit /b 1

:end
echo.
echo Build complete!
