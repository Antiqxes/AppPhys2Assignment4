#pragma once

namespace gfx {

void initDraw(int screenWidth, int screenHeight);
void shutdownDraw();

void drawBox(float x, float y, float width, float height, float angleRadians, float r, float g,
             float b);

}  // namespace gfx
