#include "draw.h"

#include <glad/glad.h>

#include <stdexcept>
#include <string>

namespace {

constexpr const char* kVertexShaderSource = R"(#version 330 core
layout (location = 0) in vec2 aPos;
uniform vec2 uScreen;
uniform vec2 uCenter;
uniform vec2 uSize;
uniform float uAngle;
void main() {
    vec2 local = aPos * uSize;
    float c = cos(uAngle);
    float s = sin(uAngle);
    vec2 pixel = uCenter + vec2(c * local.x - s * local.y, s * local.x + c * local.y);
    gl_Position = vec4(pixel.x / uScreen.x * 2.0 - 1.0, 1.0 - pixel.y / uScreen.y * 2.0, 0.0, 1.0);
}
)";

constexpr const char* kFragmentShaderSource = R"(#version 330 core
out vec4 FragColor;
uniform vec3 uColor;
void main() { FragColor = vec4(uColor, 1.0); }
)";

struct BoxRenderer {
    GLuint program = 0;
    GLuint vao = 0;
    GLuint vbo = 0;
    GLint centerLoc = -1;
    GLint sizeLoc = -1;
    GLint angleLoc = -1;
    GLint colorLoc = -1;
};

BoxRenderer gRenderer;

GLuint compileShader(GLenum type, const char* source) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint compiled = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (!compiled) {
        char log[512];
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        glDeleteShader(shader);
        throw std::runtime_error(std::string("Shader compilation failed: ") + log);
    }
    return shader;
}

GLuint createProgram(const char* vertexSource, const char* fragmentSource) {
    GLuint vertexShader = compileShader(GL_VERTEX_SHADER, vertexSource);
    GLuint fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentSource);

    GLuint program = glCreateProgram();
    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);
    glLinkProgram(program);
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    GLint linked = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &linked);
    if (!linked) {
        char log[512];
        glGetProgramInfoLog(program, sizeof(log), nullptr, log);
        glDeleteProgram(program);
        throw std::runtime_error(std::string("Program linking failed: ") + log);
    }
    return program;
}

}  // namespace

namespace gfx {

void initDraw(int screenWidth, int screenHeight) {
    gRenderer.program = createProgram(kVertexShaderSource, kFragmentShaderSource);
    gRenderer.centerLoc = glGetUniformLocation(gRenderer.program, "uCenter");
    gRenderer.sizeLoc = glGetUniformLocation(gRenderer.program, "uSize");
    gRenderer.angleLoc = glGetUniformLocation(gRenderer.program, "uAngle");
    gRenderer.colorLoc = glGetUniformLocation(gRenderer.program, "uColor");

    glUseProgram(gRenderer.program);
    glUniform2f(glGetUniformLocation(gRenderer.program, "uScreen"),
                static_cast<float>(screenWidth), static_cast<float>(screenHeight));

    constexpr float vertices[] = {
        -0.5f, -0.5f,
         0.5f, -0.5f,
        -0.5f,  0.5f,
         0.5f,  0.5f,
    };

    glGenVertexArrays(1, &gRenderer.vao);
    glGenBuffers(1, &gRenderer.vbo);

    glBindVertexArray(gRenderer.vao);
    glBindBuffer(GL_ARRAY_BUFFER, gRenderer.vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void shutdownDraw() {
    glDeleteVertexArrays(1, &gRenderer.vao);
    glDeleteBuffers(1, &gRenderer.vbo);
    glDeleteProgram(gRenderer.program);
    gRenderer = BoxRenderer{};
}

void drawBox(float x, float y, float width, float height, float angleRadians, float r, float g,
             float b) {
    glUseProgram(gRenderer.program);
    glUniform2f(gRenderer.centerLoc, x, y);
    glUniform2f(gRenderer.sizeLoc, width, height);
    glUniform1f(gRenderer.angleLoc, angleRadians);
    glUniform3f(gRenderer.colorLoc, r, g, b);
    glBindVertexArray(gRenderer.vao);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
}

}  // namespace gfx
