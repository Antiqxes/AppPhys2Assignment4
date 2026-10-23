@echo off
setlocal enabledelayedexpansion
cd /d "%~dp0.."

if not exist build\box2d_obj mkdir build\box2d_obj
if not exist lib mkdir lib

pushd build\box2d_obj
del /q *.o 2>nul

set SOURCES=
for %%d in (collision common dynamics rope) do (
    for %%f in (..\..\src\box2d\%%d\*.cpp) do set SOURCES=!SOURCES! %%f
)

echo Compiling Box2D...
g++ -c -O2 -std=c++20 -I..\..\include -I..\..\src %SOURCES%
if errorlevel 1 (
    popd
    exit /b 1
)

set OBJECTS=
for %%f in (*.o) do set OBJECTS=!OBJECTS! %%f

if exist ..\..\lib\libbox2d.a del ..\..\lib\libbox2d.a
ar rcs ..\..\lib\libbox2d.a %OBJECTS%
if errorlevel 1 (
    popd
    exit /b 1
)
popd

echo Built lib\libbox2d.a
