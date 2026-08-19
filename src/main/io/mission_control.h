/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"

#define MISSION_DEBUG_MESSAGE_SIZE 48

typedef struct missionDebugMessage_s {
    uint32_t timestampMs;
    uint8_t length;
    char text[MISSION_DEBUG_MESSAGE_SIZE];
} missionDebugMessage_t;

void missionControlInit(void);
void missionControlProcess(timeUs_t currentTimeUs);
void missionControlUpdateMode(timeUs_t currentTimeUs);

bool missionControlActive(void);
float missionControlAngleDeg(int axis);
float missionControlYawRateDps(void);
float missionControlAltitudeCm(void);

void missionDebugPrintf(const char *format, ...);
bool missionDebugPop(missionDebugMessage_t *message);
uint8_t missionDebugQueued(void);
uint32_t missionDebugDropped(void);
