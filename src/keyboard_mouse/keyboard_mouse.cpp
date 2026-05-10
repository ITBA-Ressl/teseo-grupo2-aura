/**
 * Teseo Micromouse Virtual Competition
 * Remote mouse
 *
 * @details Implements a remote-controlled mouse. The user can control the mouse using the WASD keys on the keyboard.
 * @author Theseús the hero
 */

// IMPORTANT: RENAME THIS FOLDER, FILES AND STRUCTURE TO SUIT YOUR PROJECT

#include <cmath>
#include <cstdio>

#include "raylib.h"

#include "keyboard_mouse.h"

#define LINEAR_VELOCITY 1.0f
#define ANGULAR_VELOCITY (0.5f * TURN_CCW)

static void *LoadKeyboardMouse()
{
    return nullptr;
}

static void UnloadKeyboardMouse(void *userdata)
{
}

static void UpdateKeyboardMouse(void *userdata, Sim *sim)
{
    Vector2 control = {0.0f, 0.0f};

    if (IsKeyDown(KEY_W))
        control.y += 1.0f;
    if (IsKeyDown(KEY_S))
        control.y -= 1.0f;
    if (IsKeyDown(KEY_A))
        control.x += 1.0f;
    if (IsKeyDown(KEY_D))
        control.x -= 1.0f;

    SetMouseSetpoint(sim, LINEAR_VELOCITY * control.y, ANGULAR_VELOCITY * control.x);
}

void ResetKeyboardMouse(void *userdata, Sim *sim)
{
}

MouseDescriptor keyboard_mouse = {
    "Keyboard Mouse [WASD]",
    LoadKeyboardMouse,
    UnloadKeyboardMouse,
    UpdateKeyboardMouse,
    ResetKeyboardMouse,
};
