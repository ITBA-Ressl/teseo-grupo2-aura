/**
 * Teseo Micromouse Virtual Competition
 * Agent interface
 *
 * @details Defines the agent interface for mouse implementations.
 * @author Theseús the hero
 */

#include "agent.h"

Mouse *CreateMouse(MouseDescriptor &descriptor)
{
    Mouse *instance = new Mouse();

    instance->descriptor = &descriptor;
    instance->userdata   = descriptor.create();

    return instance;
}

void DestroyMouse(Mouse *instance)
{
    instance->descriptor->destroy(instance->userdata);
    instance->userdata = nullptr;

    delete instance;
}
