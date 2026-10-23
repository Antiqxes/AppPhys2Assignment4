# Box2D Starter

A C++20 and OpenGL boilerplate for Box2D projects on Windows. It opens an
800x600 window, draws one orange rectangle, and closes when you press `Esc`.
The mouse is polled each frame and a left click prints its position to the
console. Box2D v2.4.2 is vendored and built into a static library, but nothing
uses it yet.

GLFW, GLAD and Box2D are vendored, so nothing needs to be installed beyond the
compiler.

## Prerequisites

- **MinGW-w64 via MSYS2**. Install from [msys2.org](https://www.msys2.org/),
  then install the `mingw-w64-x86_64-gcc` toolchain from the MSYS2 shell.
- **VS Code**.
- The MinGW `bin` folder (e.g. `C:\msys64\mingw64\bin`) on your system `PATH`.
  The build tasks call `g++` and `ar` by name, and the compiler's own child
  processes find their DLLs through the system `PATH`, so without this entry
  the build can fail silently.

## Build and run

1. Open the folder in VS Code.
2. Run the `Build Main` task (`Ctrl+Shift+B`). The first build also compiles
   Box2D into `lib/libbox2d.a`, which takes about 40 seconds. Later builds
   skip that step.
3. Run `main.exe` from the repo folder. Press `Esc` to close it.

## Rebuild the Box2D library

`lib/libbox2d.a` is not committed, because a static C++ library built with one
GCC version can fail to link with another. Each machine builds its own. To
rebuild it after switching compilers or editing `src/box2d/`, run the
`Build Box2D library` task.

## Project structure

```
.vscode/tasks.json        build tasks
glfw3.dll                 GLFW runtime, must sit next to main.exe
lib/libglfw3dll.a         GLFW import library
src/
  main.cpp                window, input polling, one rectangle
  draw.h/.cpp             gfx::drawBox and shader setup
  glad.c                  vendored GLAD loader
  core/                   GLFW context, window and keyboard input (gfx namespace)
  box2d/                  vendored Box2D v2.4.2 sources and its LICENSE
include/
  box2d/                  Box2D headers
  glad/, GLFW/, KHR/      vendored GLAD and GLFW headers
scripts/
  build_box2d.bat         compiles src/box2d into lib/libbox2d.a
```

## Troubleshooting

- **`g++` or `ar` is not recognized, or the build fails with no error text.**
  The MinGW `bin` folder is not on your `PATH`. Add it, then restart VS Code.
- **`cannot open output file main.exe: Permission denied`.** The program is
  still running. Close its window and build again.
- **Linker errors mentioning `box2d` or `libstdc++`.** `lib/libbox2d.a` came
  from a different compiler. Delete it or run `Build Box2D library`.
- **Program compiles but won't run.** `glfw3.dll` must sit next to `main.exe`.
