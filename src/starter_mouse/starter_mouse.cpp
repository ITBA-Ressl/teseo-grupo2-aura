/**
 * Teseo Micromouse Virtual Competition
 * Starter mouse
 *
 * @details Implements a simple right-hand wall-follower.
 * @author Theseús the hero
 */

// IMPORTANT: RENAME THIS FOLDER, FILES AND STRUCTURE TO SUIT YOUR PROJECT

#include <cmath>
#include <cstdio>

#include "starter_mouse.h"

struct AgentState
{
    bool just_turned_right = false;

    Vector2 mouse_position; // Current estimated position (meters, north/east coordinates)
    float mouse_rotation;   // Current estimated rotation (radians, CCW+, 0 = East)
};

// Wall detection: sensor distance below this → wall present.
#define WALL_DETECT_THRESHOLD 0.115f

// Step-completion thresholds.
#define DIST_DONE 0.001f // m — remaining distance below this → translation complete
#define ANGLE_DONE 0.01f // rad — remaining rotation below this → rotation complete (~3°)

void ResetStarterMouse(void *userdata, Sim *sim);

static void *LoadStarterMouse()
{
    AgentState *agent_state = new AgentState();

    ResetStarterMouse(agent_state, nullptr);

    return agent_state;
}

static void UnloadStarterMouse(void *userdata)
{
    delete (AgentState *)userdata;
}

static void UpdateStarterMouse(void *userdata, Sim *sim)
{
    AgentState *agent_state = (AgentState *)userdata;
    const SimState *sim_state = GetSimState(sim);

    // Wait for the current step to finish before deciding the next move.
    if (fabsf(sim_state->mouse_remaining_distance) > DIST_DONE ||
        fabsf(sim_state->mouse_remaining_rotation) > ANGLE_DONE)
        return;

    // Visualize visited cells in the UI.
    Cell cell = PositionToCell(agent_state->mouse_position);
    PaintCell(sim, cell, COLOR_CELL_VISITED);

    // Wall detection: sensor distance < WALL_DETECT_THRESHOLD → wall present.
    bool wall_right = sim_state->mouse_sensors[SENSOR_RIGHT] < WALL_DETECT_THRESHOLD;
    bool wall_front = sim_state->mouse_sensors[SENSOR_FRONT] < WALL_DETECT_THRESHOLD;
    bool wall_left = sim_state->mouse_sensors[SENSOR_LEFT] < WALL_DETECT_THRESHOLD;

    // Right-hand wall follower logic.
    float next_distance = 0.0f;
    float next_rotation = 0.0f;

    if (agent_state->just_turned_right)
    {
        // Advance after a turn
        agent_state->just_turned_right = false;
        next_distance = CELL_SIZE;
    }
    else if (!wall_right)
    {
        // Turn right if possible
        agent_state->just_turned_right = true;
        next_rotation = TURN_CW;
    }
    else if (!wall_front)
    {
        // Corridor ahead → go straight
        next_distance = CELL_SIZE;
    }
    else if (!wall_left)
    {
        // Turn left
        next_rotation = TURN_CCW;
    }
    else
    {
        // Dead end → reverse
        next_rotation = TURN_REVERSE;
    }

    // Update estimated position and rotation
    agent_state->mouse_rotation += next_rotation;
    agent_state->mouse_position = Vector2Add(agent_state->mouse_position,
                                             Vector2FromAngle(agent_state->mouse_rotation, next_distance));

    // Set the next setpoint for the controller
    SetMouseSetpoint(sim, next_distance, next_rotation);
}

void ResetStarterMouse(void *userdata, Sim *sim)
{
    AgentState *agent_state = (AgentState *)userdata;

    agent_state->mouse_position = {CELL_HALF_SIZE, CELL_HALF_SIZE};
    agent_state->mouse_rotation = ROTATION_NORTH;

    if (sim)
        ResetCellColors(sim);
}

MouseDescriptor starter_mouse = {
    "Starter Mouse",
    LoadStarterMouse,
    UnloadStarterMouse,
    UpdateStarterMouse,
    ResetStarterMouse,
};
