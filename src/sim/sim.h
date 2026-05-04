/**
 * Teseo Micromouse Virtual Competition
 * Physics simulation module
 *
 * @brief Implements the physics simulation using Box2D.
 * @author Theseús the hero
 */

#ifndef SIM_H
#define SIM_H

#include <cstdint>

#include <raymath.h>

#include "maze.h"

// Rotations

#define PI 3.14159265358979323846f

#define ROTATION_EAST (0.0f * PI)
#define ROTATION_NORTH (0.5f * PI)
#define ROTATION_WEST (1.0f * PI)
#define ROTATION_SOUTH (1.5f * PI)

#define TURN_CCW (0.5f * PI) // -90° turn in radians
#define TURN_CW (-0.5f * PI) // +90° turn in radians
#define TURN_REVERSE PI      // +180° turn in radians

// Maze geometry

#define CELL_SIZE 0.18f       // meters
#define WALL_THICKNESS 0.012f // meters

#define CELL_HALF_SIZE (CELL_SIZE / 2.0f)
#define WALL_HALF_THICKNESS (WALL_THICKNESS / 2.0f)
#define WALL_HALF_WIDTH (CELL_SIZE / 2.0f + WALL_HALF_THICKNESS)
#define WALL_HALF_HEIGHT (CELL_SIZE / 2.0f + WALL_HALF_THICKNESS)

// Mouse geometry and physics

#define MOUSE_WIDTH 0.080f       // meters
#define MOUSE_LENGTH 0.100f      // meters
#define MOUSE_MASS 0.100f        // kg
#define MOUSE_WHEEL_TRACK 0.070f // meters (axle width)

#define MOUSE_HALF_WIDTH (MOUSE_WIDTH / 2.0f)
#define MOUSE_HALF_LENGTH (MOUSE_LENGTH / 2.0f)
#define MOUSE_DENSITY (MOUSE_MASS / (MOUSE_HALF_WIDTH * MOUSE_HALF_LENGTH))
#define MOUSE_WHEEL_HALF_TRACK (MOUSE_WHEEL_TRACK / 2.0f)

// Mouse controller

#define MOUSE_WHEEL_VELOCITY_MAX 1.5f // m/s
#define MOUSE_WHEEL_FORCE_MAX 0.5f    // N per wheel
#define MOUSE_MOTOR_K 6.0f            // N/(m/s) — velocity → force gain
#define MOUSE_KP_ROTATION 8.0f        // rad/s per rad of error (rotation)
#define MOUSE_KP_DISTANCE 8.0f        // m/s per m of error (distance)

#define WHEEL_ANGULAR_VELOCITY_MAX (2.0f * MOUSE_WHEEL_VELOCITY_MAX / MOUSE_WHEEL_TRACK)

// Sensor noise
#define SETPOINT_DISTANCE_NOISE 0.01f  // factor of target distance
#define SETPOINT_ROTATION_NOISE 0.01f  // radians

// Mouse sensors (5 IR sensors: left, front-left, front, front-right, right)

#define SENSOR_NUM 5
#define SENSOR_RANGE_MIN MOUSE_HALF_WIDTH // meters
#define SENSOR_RANGE_MAX 1.0f             // meters

enum
{
    SENSOR_LEFT,
    SENSOR_FRONT_LEFT,
    SENSOR_FRONT,
    SENSOR_FRONT_RIGHT,
    SENSOR_RIGHT,
};

extern float SENSOR_ANGLES[SENSOR_NUM];

// Sim state

#define RUN_TOTAL 5         // Total number of runs to execute
#define RUN_TIME_MAX 300.0f // Seconds

enum RunState
{
    RUNSTATE_IDLE,
    RUNSTATE_RUNNING,
    RUNSTATE_RETURNING,
};

/**
 * @brief The state of the simulation at a given moment, used for rendering and by the mouse agent.
 */
struct SimState
{
    float time;          // Elapsed time since simulation start (seconds).
    int run_number;      // Run counter (starts at 1, increments on reset).
    RunState run_state;  // Current run state (idle, running, returning).
    float run_time;      // Elapsed time since start of run (seconds).
    float run_time_best; // Best time achieved so far (seconds).

    float mouse_sensors[SENSOR_NUM]; // Distance readings from the 5 IR sensors (meters)
    float mouse_remaining_distance;  // Distance remaining to reach setpoint (meters, positive = forward)
    float mouse_remaining_rotation;  // Rotation remaining to reach setpoint (radians, CCW+)
};

// Opaque handle

struct Sim;

// -----------------------------------------------------------------------------
// WARNING: Only the following functions are available to mouse agents.
// Do not call any other functions from the simulation API.
// Doing so will disqualify yout team from the competition.

/**
 * @brief Creates a vector from an angle in radians (CCW+).
 *
 * @param angle The angle in radians (CCW+).
 * @param length Optional length of the resulting vector (default is 1.0f).
 *
 * @return A vector pointing in the direction of the angle.
 */
Vector2 Vector2FromAngle(float angle, float length = 1.0f);

/**
 * @brief Computes the difference between two angles, returning the smallest signed angle between them.
 *
 * @param source The source angle (radians).
 * @param target The target angle (radians).
 *
 * @return The angle difference (radians, in the range [-π, π], positive = CCW).
 */
float AngleDiff(float source, float target);

/**
 * @brief Converts a world position (meters) to a maze cell coordinate.
 *
 * @param position The world position to convert (meters, north/east coordinates).
 *
 * @return The corresponding maze cell coordinate.
 */
Cell PositionToCell(Vector2 position);

/**
 * @brief Paints the given maze cell with the specified color. Used for debugging and visualization.
 *
 * @param sim The simulation instance.
 * @param cell The cell coordinate to paint.
 * @param color The color to use for painting.
 */
void PaintCell(Sim *sim, Cell cell, uint32_t color);

/**
 * @brief Resets the colors of all maze cells to the default color. Used for debugging and visualization.
 *
 * @param sim The simulation instance.
 */
void ResetCellColors(Sim *sim);

/**
 * @brief Gets the color associated with the given maze cell, based on its state (e.g. discovered, goal).
 *
 * @param sim The simulation instance.
 * @param cell The cell coordinate to query.
 *
 * @return The color associated with the maze cell.
 */
uint32_t GetCellColor(Sim *sim, Cell cell);

/**
 * @brief Returns the current state of the simulation.
 *
 * @param sim The simulation instance to query.
 *
 * @return A pointer to the current simulation state.
 */
const SimState *GetSimState(Sim *sim);

/**
 * @brief Sets a movement setpoint for the mouse.
 *        The mouse will attempt to rotate to the target rotation and drive to the target distance.
 *
 * @param sim The simulation instance.
 * @param distance The distance setpoint in meters (positive = forward).
 * @param rotation The rotation setpoint in radians (CCW+).
 */
void SetMouseSetpoint(Sim *sim, float distance, float rotation);

// This is the end of the agent API.
// The following functions are used by the UI and must not be called by mouse agents.
// -----------------------------------------------------------------------------

// Simulation API

/**
 * @brief Creates a new Teseo simulation instance with the given maze.
 *
 * @param maze The maze layout to use for the simulation.
 *
 * @return A pointer to the created simulation instance.
 */
Sim *CreateSim(const Maze *maze);

/**
 * @brief Frees all resources associated with the simulation instance.
 *
 * @param sim The simulation instance to destroy.
 */
void DestroySim(Sim *sim);

// Simulation

/**
 * @brief Checks if the simulation is currently running (i.e. a run is ongoing and not yet completed).
 *
 * @return true if the simulation is running, false otherwise.
 */
bool IsSimRunning(Sim *sim);

/**
 * @brief Gets the current position of the mouse (meters, north/east).
 *
 * @param sim The simulation instance to query.
 *
 * @return The current position of the mouse in world coordinates.
 */
Vector2 GetMousePosition(Sim *sim);

/**
 * @brief Gets the current rotation of the mouse in radians (CCW+).
 *
 * @param sim The simulation instance to query.
 *
 * @return The current rotation of the mouse in radians.
 */
float GetMouseRotation(Sim *sim);

/**
 * @brief Starts a new run and resets the mouse.
 *
 * @param sim The simulation instance to reset.
 *
 * @return true if the reset was successful and a new run has started,
 *         false if the maximum number of runs has been reached.
 */
bool ResetSim(Sim *sim);

/**
 * @brief Steps the physics simulation forward by dt seconds.
 *
 * @param sim The simulation instance to step.
 * @param dt Time step in seconds.
 */
void UpdateSim(Sim *sim, float dt);

#endif // SIM_H
