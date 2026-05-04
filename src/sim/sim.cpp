/**
 * Teseo Micromouse Virtual Competition
 * Physics simulation module
 *
 * @brief Implements the physics simulation using Box2D.
 * @author Theseús the hero
 */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

#include <box2d/box2d.h>

#include "sim.h"

// Sensor angles

float SENSOR_ANGLES[5] = {
    TURN_CCW,        // Left       (90° left)
    TURN_CCW / 2.0f, // Front-left (45° left)
    0,               // Front
    TURN_CW / 2.0f,  // Front-right (45° right)
    TURN_CW,         // Right      (90° right)
};

// Sim state

struct Sim
{
    // Maze
    const Maze *maze;
    uint32_t maze_colors[GRID_SIZE][GRID_SIZE];

    // Box2D world and bodies
    b2WorldId world;
    b2BodyId mouse_body;

    // Sim state
    Vector2 mouse_position; // World position (meters, north/east coordinates)
    float mouse_rotation;   // Rotation (radians, CCW+, 0 = East)

    SimState state;

    // Mouse controller
    Vector2 setpoint_reference_position; // Reference position when setpoint was issued
    float setpoint_reference_rotation;   // Reference rotation when setpoint was issued
    float setpoint_target_distance;      // Target distance
    float setpoint_target_rotation;      // Target rotation
};

// Math helpers

Vector2 Vector2FromAngle(float angle, float length)
{
    return Vector2{cosf(angle) * length, sinf(angle) * length};
}

float AngleDiff(float source, float target)
{
    float diff = target - source;

    diff = atan2f(sinf(diff), cosf(diff));

    return diff;
}

// Maze

Cell PositionToCell(Vector2 position)
{
    return {
        (int32_t)floorf(position.x / CELL_SIZE),
        (int32_t)floorf(position.y / CELL_SIZE)};
}

void PaintCell(Sim *sim, Cell cell, uint32_t color)
{
    if (!ValidateCell(cell))
        return;

    sim->maze_colors[cell.x][cell.y] = color;
}

uint32_t GetCellColor(Sim *sim, Cell cell)
{
    if (!ValidateCell(cell))
        return COLOR_CELL_DEFAULT;

    return sim->maze_colors[cell.x][cell.y];
}

void ResetCellColors(Sim *sim)
{
    for (int x = 0; x < GRID_SIZE; x++)
        for (int y = 0; y < GRID_SIZE; y++)
            PaintCell(sim, {x, y}, COLOR_CELL_DEFAULT);
}

static void CreateMazePhysics(Sim *sim)
{
    // One static body at the origin.
    b2BodyDef body_def = b2DefaultBodyDef();
    body_def.type = b2_staticBody;
    b2BodyId walls_body = b2CreateBody(sim->world, &body_def);

    b2ShapeDef shape_def = b2DefaultShapeDef();
    shape_def.material.restitution = 0.05f; // Nearly inelastic (foam-tipped ABS walls)
    shape_def.material.friction = 0.4f;

    // Horizontal wall segments
    for (int32_t y = 0; y <= GRID_SIZE; y++)
    {
        for (int32_t x = 0; x < GRID_SIZE; x++)
        {
            bool has_wall;
            if (y == 0)
                has_wall = HasWall(sim->maze, {x, y}, WALL_SOUTH);
            else
                has_wall = HasWall(sim->maze, {x, y - 1}, WALL_NORTH);

            if (has_wall)
            {
                Vector2 center = {(x + 0.5f) * CELL_SIZE, y * CELL_SIZE};

                b2Polygon box = b2MakeOffsetBox(WALL_HALF_WIDTH, WALL_HALF_THICKNESS,
                                                b2Vec2(center.x, center.y), b2MakeRot(0.0f));
                b2CreatePolygonShape(walls_body, &shape_def, &box);
            }
        }
    }

    // Vertical wall segments
    for (int32_t x = 0; x <= GRID_SIZE; x++)
    {
        for (int32_t y = 0; y < GRID_SIZE; y++)
        {
            bool has_wall;
            if (x == 0)
                has_wall = HasWall(sim->maze, {x, y}, WALL_WEST);
            else
                has_wall = HasWall(sim->maze, {x - 1, y}, WALL_EAST);

            if (has_wall)
            {
                Vector2 center = {x * CELL_SIZE, (y + 0.5f) * CELL_SIZE};

                b2Polygon box = b2MakeOffsetBox(WALL_HALF_THICKNESS, WALL_HALF_HEIGHT,
                                                b2Vec2(center.x, center.y), b2MakeRot(0.0f));
                b2CreatePolygonShape(walls_body, &shape_def, &box);
            }
        }
    }
}

// Mouse

static void CreateMousePhysics(Sim *sim)
{
    b2BodyDef body_def = b2DefaultBodyDef();
    body_def.type = b2_dynamicBody;
    body_def.position = b2Vec2(0.5f * CELL_SIZE, 0.5f * CELL_SIZE); // Start in center of cell (0,0)
    body_def.rotation = b2MakeRot(ROTATION_NORTH);
    body_def.linearDamping = 2.0f; // Floor friction (s⁻¹)
    body_def.angularDamping = 3.0f;
    body_def.fixedRotation = false;
    sim->mouse_body = b2CreateBody(sim->world, &body_def);

    b2ShapeDef shape_def = b2DefaultShapeDef();
    shape_def.density = MOUSE_DENSITY;
    shape_def.material.friction = 0.2f;
    shape_def.material.restitution = 0.05f;

    b2Polygon box = b2MakeBox(MOUSE_HALF_WIDTH, MOUSE_HALF_LENGTH);
    b2CreatePolygonShape(sim->mouse_body, &shape_def, &box);
}

static void UpdateMouseState(Sim *sim)
{
    // Kinematic state update
    b2Vec2 position = b2Body_GetPosition(sim->mouse_body);
    float rotation = b2Rot_GetAngle(b2Body_GetRotation(sim->mouse_body));
    b2Vec2 velocity = b2Body_GetLinearVelocity(sim->mouse_body);
    float angular_velocity = b2Body_GetAngularVelocity(sim->mouse_body);

    sim->mouse_position = {position.x, position.y};
    sim->mouse_rotation = rotation;

    // Sensor state update
    b2QueryFilter filter = b2DefaultQueryFilter();

    for (int i = 0; i < SENSOR_NUM; i++)
    {
        float ray_angle = rotation + SENSOR_ANGLES[i];
        Vector2 direction = Vector2FromAngle(ray_angle, SENSOR_RANGE_MAX);

        b2Vec2 start = b2Vec2{position.x, position.y};
        b2Vec2 translation = b2Vec2{direction.x, direction.y};

        b2RayResult rayResult = b2World_CastRayClosest(sim->world, start, translation, filter);

        if (rayResult.hit)
            sim->state.mouse_sensors[i] = rayResult.fraction * SENSOR_RANGE_MAX;
        else
            sim->state.mouse_sensors[i] = SENSOR_RANGE_MAX;
    }
}

static void ResetMousePhysics(Sim *sim)
{
    // Reset mouse state
    Vector2 position = {0.5f * CELL_SIZE, 0.5f * CELL_SIZE};
    float rotation = ROTATION_NORTH;

    b2Body_SetTransform(sim->mouse_body, b2Vec2(position.x, position.y), b2MakeRot(rotation));
    b2Body_SetLinearVelocity(sim->mouse_body, b2Vec2(0.0f, 0.0f));
    b2Body_SetAngularVelocity(sim->mouse_body, 0.0f);

    UpdateMouseState(sim);

    // Reset controller
    sim->setpoint_reference_position = position;
    sim->setpoint_reference_rotation = rotation;
    sim->setpoint_target_distance = 0.0f;
    sim->setpoint_target_rotation = rotation;
}

// Controller

static void ApplyDrive(Sim *sim, float velocity_left_desired, float velocity_right_desired)
{
    b2Vec2 velocity = b2Body_GetLinearVelocity(sim->mouse_body);
    float rotation = b2Rot_GetAngle(b2Body_GetRotation(sim->mouse_body));
    float angular_velocity = b2Body_GetAngularVelocity(sim->mouse_body);

    // Body-frame forward unit vector
    float fwd_x = std::cos(rotation);
    float fwd_y = std::sin(rotation);

    // Forward speed in body frame
    float v_fwd = velocity.x * fwd_x + velocity.y * fwd_y;

    // Actual wheel speeds
    float velocity_left_current = v_fwd - angular_velocity * MOUSE_WHEEL_HALF_TRACK;
    float velocity_right_current = v_fwd + angular_velocity * MOUSE_WHEEL_HALF_TRACK;

    // Motor force (velocity P-control per wheel, clamped to motor limit)
    float force_left = std::clamp(MOUSE_MOTOR_K * (velocity_left_desired - velocity_left_current),
                                  -MOUSE_WHEEL_FORCE_MAX, MOUSE_WHEEL_FORCE_MAX);
    float force_right = std::clamp(MOUSE_MOTOR_K * (velocity_right_desired - velocity_right_current),
                                   -MOUSE_WHEEL_FORCE_MAX, MOUSE_WHEEL_FORCE_MAX);

    // Net force (forward) and torque (CCW)
    float force_net = force_left + force_right;
    float tau = (force_right - force_left) * MOUSE_WHEEL_HALF_TRACK;

    b2Body_ApplyForceToCenter(sim->mouse_body, b2Vec2(force_net * fwd_x, force_net * fwd_y), true);
    b2Body_ApplyTorque(sim->mouse_body, tau, true);

    // Lateral no-slip impulse: cancel velocity perpendicular to heading.
    // Left unit vector (CCW 90° from forward): (-fwd_y, fwd_x)
    float lat_vel = velocity.x * (-fwd_y) + velocity.y * fwd_x;
    float lat_impulse = -MOUSE_MASS * lat_vel;
    b2Body_ApplyLinearImpulseToCenter(sim->mouse_body,
                                      b2Vec2(lat_impulse * (-fwd_y), lat_impulse * fwd_x),
                                      true);
}

static void UpdateMouseController(Sim *sim)
{
    Vector2 position = sim->mouse_position;
    float rotation = sim->mouse_rotation;

    // Rotation error
    float rotation_error = AngleDiff(rotation, sim->setpoint_target_rotation);

    // Distance error
    Vector2 position_error = Vector2Subtract(position, sim->setpoint_reference_position);
    Vector2 forward = Vector2FromAngle(rotation);
    // Projects position_error onto forward direction
    float distance_current = Vector2DotProduct(position_error, forward);
    float distance_error = sim->setpoint_target_distance - distance_current;

    // P-control: simultaneous forward speed + angular rate
    float velocity_desired = std::clamp(MOUSE_KP_DISTANCE * distance_error,
                                        -MOUSE_WHEEL_VELOCITY_MAX, MOUSE_WHEEL_VELOCITY_MAX);
    float angular_velocity_desired = std::clamp(MOUSE_KP_ROTATION * rotation_error,
                                                -WHEEL_ANGULAR_VELOCITY_MAX, WHEEL_ANGULAR_VELOCITY_MAX);

    // Differential drive wheel speeds
    float velocity_left = std::clamp(velocity_desired - angular_velocity_desired * MOUSE_WHEEL_HALF_TRACK,
                                     -MOUSE_WHEEL_VELOCITY_MAX, MOUSE_WHEEL_VELOCITY_MAX);
    float velocity_right = std::clamp(velocity_desired + angular_velocity_desired * MOUSE_WHEEL_HALF_TRACK,
                                      -MOUSE_WHEEL_VELOCITY_MAX, MOUSE_WHEEL_VELOCITY_MAX);

    sim->state.mouse_remaining_distance = distance_error;
    sim->state.mouse_remaining_rotation = rotation_error;

    ApplyDrive(sim, velocity_left, velocity_right);
}

// Public API

Sim *CreateSim(const Maze *maze)
{
    Sim *sim = new Sim();
    sim->maze = maze;

    ResetCellColors(sim);

    b2WorldDef wd = b2DefaultWorldDef();
    wd.gravity = b2Vec2(0.0f, 0.0f);
    sim->world = b2CreateWorld(&wd);

    CreateMazePhysics(sim);
    CreateMousePhysics(sim);

    ResetMousePhysics(sim);

    UpdateMouseState(sim);

    return sim;
}

void DestroySim(Sim *sim)
{
    b2DestroyWorld(sim->world);

    delete sim;
}

bool ResetSim(Sim *sim)
{
    if (sim->state.run_number >= RUN_TOTAL)
        return false;

    // Reset run state
    sim->state.run_number += 1;
    sim->state.run_state = RUNSTATE_IDLE;
    sim->state.run_time = 0.0f;

    ResetMousePhysics(sim);

    return true;
}

bool IsSimRunning(Sim *sim)
{
    if (sim->state.run_number == 0)
        return false;

    if (sim->state.run_number >= RUN_TOTAL && sim->state.run_state == RUNSTATE_RETURNING)
        return false;

    if (sim->state.time >= RUN_TIME_MAX)
        return false;

    return true;
}

void UpdateSim(Sim *sim, float dt)
{
    // Update timers
    if (!(sim->state.run_number == 1 && sim->state.run_state == RUNSTATE_IDLE))
        sim->state.time += dt;

    if (sim->state.run_state == RUNSTATE_IDLE)
        sim->state.run_time = 0;
    else if (sim->state.run_state == RUNSTATE_RUNNING)
        sim->state.run_time += dt;

    // Update mouse controller
    UpdateMouseController(sim);

    // Step physics
    b2World_Step(sim->world, dt, 8);

    // Update mouse state
    UpdateMouseState(sim);

    Cell cell = PositionToCell(sim->mouse_position);

    // Check start position
    if (sim->state.run_state == RUNSTATE_IDLE)
    {
        if (!isStartCell(cell))
            sim->state.run_state = RUNSTATE_RUNNING;
    }

    // Check goal position
    if (IsGoalCell(cell))
    {
        sim->state.run_state = RUNSTATE_RETURNING;

        if (sim->state.run_time < sim->state.run_time_best || sim->state.run_time_best == 0.0f)
            sim->state.run_time_best = sim->state.run_time;
    }
}

Vector2 GetMousePosition(Sim *sim)
{
    return sim->mouse_position;
}

float GetMouseRotation(Sim *sim)
{
    return sim->mouse_rotation;
}

const SimState *GetSimState(Sim *sim)
{
    return &sim->state;
}

void SetMouseSetpoint(Sim *sim, float distance, float rotation)
{
    // Add mechanical noise
    static std::mt19937 rng(std::random_device{}());
    static std::normal_distribution<float> dist(0.0f, 1.0f);
    distance += dist(rng) * SETPOINT_DISTANCE_NOISE * distance;
    rotation += dist(rng) * SETPOINT_ROTATION_NOISE;

    sim->setpoint_reference_position = sim->mouse_position;
    sim->setpoint_reference_rotation = sim->mouse_rotation;

    sim->setpoint_target_distance = distance;
    sim->setpoint_target_rotation = sim->mouse_rotation + rotation;

    sim->state.mouse_remaining_distance = distance;
    sim->state.mouse_remaining_rotation = AngleDiff(sim->mouse_rotation, sim->setpoint_target_rotation);
}
