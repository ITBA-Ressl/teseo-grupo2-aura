/**
 * Teseo Micromouse Virtual Competition
 * UI module
 *
 * @brief Provides functions to create a raylib window, render the maze and mouse state,
 *        and handle user input (reset command).
 * @author Theseús the hero
 */

#ifndef UI_H
#define UI_H

#include "../sim/agent.h"

/**
 * @brief Creates the raylib window and initializes UI state.
 *
 * @param maze The maze layout, used for rendering the walls and goal.
 * @param mouse The mouse instance, used to access mouse state for rendering.
 */
void CreateUI(const Maze *maze, Mouse *mouse);

/**
 * @brief Steps the simulation and updates the UI.
 *        Returns false when the window should close.
 *        Also handles user input for resetting the simulation.
 * 
 * @return true if the UI should continue running, false to exit.
 */
bool UpdateUI();

/**
 * @brief Destroys the raylib window and frees UI resources.
 */
void DestroyUI();

#endif // UI_H
