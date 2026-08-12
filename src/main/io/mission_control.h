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

#include "common/time.h"

void missionControlInit(void);
void missionControlProcess(timeUs_t currentTimeUs);
void missionControlUpdateMode(timeUs_t currentTimeUs);

bool missionControlActive(void);
float missionControlAngleDeg(int axis);
float missionControlYawRateDps(void);
float missionControlAltitudeCm(void);
