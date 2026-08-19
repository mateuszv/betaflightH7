/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "platform.h"

#ifdef USE_MISSION_CONTROL

#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>

#include "common/axis.h"
#include "common/crc.h"
#include "common/maths.h"
#include "common/printf.h"
#include "drivers/serial.h"
#include "drivers/time.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "flight/imu.h"
#include "flight/position.h"
#include "io/serial.h"

#include "io/mission_control.h"

#define MISSION_BAUDRATE                 115200
#define MISSION_SYNC_1                   0xA5
#define MISSION_SYNC_2                   0x5A
#define MISSION_PROTOCOL_VERSION         1
#define MISSION_MAX_PAYLOAD_SIZE         32
#define MISSION_HEADER_SIZE              5
#define MISSION_HEARTBEAT_INTERVAL_US    1000000
#define MISSION_HEARTBEAT_TIMEOUT_US     3000000
#define MISSION_SETPOINT_TIMEOUT_US      500000
#define MISSION_HEADING_TIMEOUT_US       500000
#define MISSION_MAX_ANGLE_CDEG           6000
#define MISSION_MAX_ALTITUDE_CM          100000
#define MISSION_YAW_P                    2.0f
#define MISSION_MAX_YAW_RATE_DPS         180.0f
#define MISSION_DEBUG_QUEUE_LENGTH       32

typedef enum {
    MISSION_MSG_HEARTBEAT = 1,
    MISSION_MSG_SETPOINT = 2,
    MISSION_MSG_HEADING_UPDATE = 3,
} missionMessageType_e;

typedef enum {
    PARSER_SYNC_1,
    PARSER_SYNC_2,
    PARSER_HEADER,
    PARSER_PAYLOAD,
    PARSER_CRC_LOW,
    PARSER_CRC_HIGH,
} missionParserState_e;

typedef struct {
    int16_t rollCdeg;
    int16_t pitchCdeg;
    uint16_t targetYawCdeg;
    int32_t altitudeCm;
} missionSetpoint_t;

typedef struct {
    uint16_t headingCdeg;
    uint16_t accuracyCdeg;
    int16_t localYawDeciDeg;
} missionHeading_t;

static serialPort_t *missionPort;
static missionParserState_e parserState;
static uint8_t header[MISSION_HEADER_SIZE];
static uint8_t payload[MISSION_MAX_PAYLOAD_SIZE];
static uint8_t headerPosition;
static uint8_t payloadPosition;
static uint16_t receivedCrc;

static missionSetpoint_t setpoint;
static missionHeading_t heading;
static timeUs_t lastHeartbeatUs;
static timeUs_t lastSetpointUs;
static timeUs_t lastHeadingUs;
static timeUs_t lastHeartbeatTxUs;
static uint16_t txSequence;
static uint16_t lastRxSequence;
static bool haveRxSequence;
static bool heartbeatSeenSinceEnable;
static bool setpointSeenSinceEnable;
static bool headingSeenSinceEnable;
static bool missionSwitchWasActive;
static bool controlActive;
static bool wasArmed;
static float armAltitudeCm;

static missionDebugMessage_t debugQueue[MISSION_DEBUG_QUEUE_LENGTH];
static uint8_t debugQueueHead;
static uint8_t debugQueueTail;
static uint8_t debugQueueCount;
static uint32_t debugDroppedCount;

typedef struct {
    char *buffer;
    uint8_t length;
} missionDebugFormatContext_t;

static void missionDebugPutChar(void *context, char character)
{
    missionDebugFormatContext_t *format = context;
    if (format->length < MISSION_DEBUG_MESSAGE_SIZE - 1) {
        format->buffer[format->length++] = character;
    }
}

void missionDebugPrintf(const char *format, ...)
{
    if (!format) {
        return;
    }

    if (debugQueueCount >= MISSION_DEBUG_QUEUE_LENGTH) {
        debugDroppedCount++;
        return;
    }

    missionDebugMessage_t *message = &debugQueue[debugQueueHead];
    missionDebugFormatContext_t formatContext = {
        .buffer = message->text,
        .length = 0,
    };

    va_list arguments;
    va_start(arguments, format);
    tfp_format(&formatContext, missionDebugPutChar, format, arguments);
    va_end(arguments);

    message->text[formatContext.length] = '\0';
    message->length = formatContext.length;
    message->timestampMs = millis();

    debugQueueHead = (debugQueueHead + 1) % MISSION_DEBUG_QUEUE_LENGTH;
    debugQueueCount++;
}

bool missionDebugPop(missionDebugMessage_t *message)
{
    if (!message || debugQueueCount == 0) {
        return false;
    }

    *message = debugQueue[debugQueueTail];
    debugQueueTail = (debugQueueTail + 1) % MISSION_DEBUG_QUEUE_LENGTH;
    debugQueueCount--;
    return true;
}

uint8_t missionDebugQueued(void)
{
    return debugQueueCount;
}

uint32_t missionDebugDropped(void)
{
    return debugDroppedCount;
}

static uint16_t readU16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t readI16(const uint8_t *data)
{
    return (int16_t)readU16(data);
}

static int32_t readI32(const uint8_t *data)
{
    return (int32_t)((uint32_t)data[0]
        | ((uint32_t)data[1] << 8)
        | ((uint32_t)data[2] << 16)
        | ((uint32_t)data[3] << 24));
}

static void writeU16(uint8_t *data, uint16_t value)
{
    data[0] = value & 0xff;
    data[1] = value >> 8;
}

static void writeU32(uint8_t *data, uint32_t value)
{
    data[0] = value & 0xff;
    data[1] = (value >> 8) & 0xff;
    data[2] = (value >> 16) & 0xff;
    data[3] = value >> 24;
}

static float wrapHeadingErrorDeg(float errorDeg)
{
    while (errorDeg > 180.0f) {
        errorDeg -= 360.0f;
    }
    while (errorDeg < -180.0f) {
        errorDeg += 360.0f;
    }
    return errorDeg;
}

static bool sequenceIsNew(uint16_t sequence)
{
    if (!haveRxSequence) {
        haveRxSequence = true;
        lastRxSequence = sequence;
        return true;
    }

    const uint16_t delta = sequence - lastRxSequence;
    if (delta == 0 || delta >= 0x8000) {
        return false;
    }

    lastRxSequence = sequence;
    return true;
}

static void missionComputerFailsafe(void)
{
    // TODO: define the in-flight response to loss of the mission computer.
}

static void handleFrame(timeUs_t currentTimeUs)
{
    const uint8_t type = header[1];
    const uint8_t length = header[2];
    const uint16_t sequence = readU16(&header[3]);

    if (header[0] != MISSION_PROTOCOL_VERSION || !sequenceIsNew(sequence)) {
        return;
    }

    const bool missionSwitchActive = IS_RC_MODE_ACTIVE(BOXMISSION);

    switch (type) {
    case MISSION_MSG_HEARTBEAT:
        if (length == 8) {
            lastHeartbeatUs = currentTimeUs;
            heartbeatSeenSinceEnable |= missionSwitchActive;
        }
        break;

    case MISSION_MSG_SETPOINT:
        if (length == 10) {
            const int16_t rollCdeg = readI16(&payload[0]);
            const int16_t pitchCdeg = readI16(&payload[2]);
            const uint16_t targetYawCdeg = readU16(&payload[4]);
            const int32_t altitudeCm = readI32(&payload[6]);

            if (rollCdeg >= -MISSION_MAX_ANGLE_CDEG && rollCdeg <= MISSION_MAX_ANGLE_CDEG
                && pitchCdeg >= -MISSION_MAX_ANGLE_CDEG && pitchCdeg <= MISSION_MAX_ANGLE_CDEG
                && targetYawCdeg < 36000
                && altitudeCm >= 0 && altitudeCm <= MISSION_MAX_ALTITUDE_CM) {
                setpoint.rollCdeg = rollCdeg;
                setpoint.pitchCdeg = pitchCdeg;
                setpoint.targetYawCdeg = targetYawCdeg;
                setpoint.altitudeCm = altitudeCm;
                lastSetpointUs = currentTimeUs;
                setpointSeenSinceEnable |= missionSwitchActive;
            }
        }
        break;

    case MISSION_MSG_HEADING_UPDATE:
        if (length == 4) {
            const uint16_t headingCdeg = readU16(&payload[0]);
            const uint16_t accuracyCdeg = readU16(&payload[2]);

            if (headingCdeg < 36000 && accuracyCdeg <= 18000) {
                heading.headingCdeg = headingCdeg;
                heading.accuracyCdeg = accuracyCdeg;
                heading.localYawDeciDeg = attitude.values.yaw;
                lastHeadingUs = currentTimeUs;
                headingSeenSinceEnable |= missionSwitchActive;
            }
        }
        break;

    default:
        break;
    }
}

static void parserReset(void)
{
    parserState = PARSER_SYNC_1;
    headerPosition = 0;
    payloadPosition = 0;
}

static void parseByte(uint8_t data, timeUs_t currentTimeUs)
{
    switch (parserState) {
    case PARSER_SYNC_1:
        if (data == MISSION_SYNC_1) {
            parserState = PARSER_SYNC_2;
        }
        break;

    case PARSER_SYNC_2:
        if (data == MISSION_SYNC_2) {
            parserState = PARSER_HEADER;
            headerPosition = 0;
        } else {
            parserState = (data == MISSION_SYNC_1) ? PARSER_SYNC_2 : PARSER_SYNC_1;
        }
        break;

    case PARSER_HEADER:
        header[headerPosition++] = data;
        if (headerPosition == MISSION_HEADER_SIZE) {
            if (header[2] > MISSION_MAX_PAYLOAD_SIZE) {
                parserReset();
            } else if (header[2] == 0) {
                parserState = PARSER_CRC_LOW;
            } else {
                payloadPosition = 0;
                parserState = PARSER_PAYLOAD;
            }
        }
        break;

    case PARSER_PAYLOAD:
        payload[payloadPosition++] = data;
        if (payloadPosition == header[2]) {
            parserState = PARSER_CRC_LOW;
        }
        break;

    case PARSER_CRC_LOW:
        receivedCrc = data;
        parserState = PARSER_CRC_HIGH;
        break;

    case PARSER_CRC_HIGH: {
        receivedCrc |= (uint16_t)data << 8;
        uint16_t calculatedCrc = crc16_ccitt_update(0xffff, header, sizeof(header));
        calculatedCrc = crc16_ccitt_update(calculatedCrc, payload, header[2]);
        if (calculatedCrc == receivedCrc) {
            handleFrame(currentTimeUs);
        }
        parserReset();
        break;
    }
    }
}

static void sendFrame(uint8_t type, const uint8_t *framePayload, uint8_t length)
{
    if (!missionPort || length > MISSION_MAX_PAYLOAD_SIZE) {
        return;
    }

    uint8_t frame[2 + MISSION_HEADER_SIZE + MISSION_MAX_PAYLOAD_SIZE + 2];
    frame[0] = MISSION_SYNC_1;
    frame[1] = MISSION_SYNC_2;
    frame[2] = MISSION_PROTOCOL_VERSION;
    frame[3] = type;
    frame[4] = length;
    writeU16(&frame[5], txSequence++);
    if (length) {
        memcpy(&frame[7], framePayload, length);
    }

    uint16_t crc = crc16_ccitt_update(0xffff, &frame[2], MISSION_HEADER_SIZE + length);
    writeU16(&frame[7 + length], crc);
    serialWriteBuf(missionPort, frame, 2 + MISSION_HEADER_SIZE + length + 2);
}

static void sendHeartbeat(timeUs_t currentTimeUs)
{
    uint8_t heartbeat[8] = { 0 };
    writeU32(&heartbeat[0], millis());
    heartbeat[4] = (ARMING_FLAG(ARMED) ? 1 : 0) | (controlActive ? 2 : 0);
    sendFrame(MISSION_MSG_HEARTBEAT, heartbeat, sizeof(heartbeat));
    lastHeartbeatTxUs = currentTimeUs;
}

void missionControlInit(void)
{
    debugQueueHead = 0;
    debugQueueTail = 0;
    debugQueueCount = 0;
    debugDroppedCount = 0;
    parserReset();
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_MISSION);
    if (portConfig) {
        missionPort = openSerialPort(portConfig->identifier, FUNCTION_MISSION,
            NULL, NULL, MISSION_BAUDRATE, MODE_RXTX,
            SERIAL_STOPBITS_1 | SERIAL_PARITY_NO);
    }
}

void missionControlProcess(timeUs_t currentTimeUs)
{
    if (!missionPort) {
        return;
    }

    uint32_t waiting = serialRxBytesWaiting(missionPort);
    while (waiting--) {
        parseByte(serialRead(missionPort), currentTimeUs);
    }

    if (cmpTimeUs(currentTimeUs, lastHeartbeatTxUs) >= MISSION_HEARTBEAT_INTERVAL_US) {
        sendHeartbeat(currentTimeUs);
    }
}

void missionControlUpdateMode(timeUs_t currentTimeUs)
{
    const bool wasControlActive = controlActive;
    const bool armed = ARMING_FLAG(ARMED);
    if (armed && !wasArmed) {
        armAltitudeCm = getAltitudeCmControl();
    }
    wasArmed = armed;

    const bool missionSwitchActive = IS_RC_MODE_ACTIVE(BOXMISSION);
    if (missionSwitchActive && !missionSwitchWasActive) {
        heartbeatSeenSinceEnable = false;
        setpointSeenSinceEnable = false;
        headingSeenSinceEnable = false;
        controlActive = false;
        missionDebugPrintf("MISSION switch on");
    } else if (!missionSwitchActive && missionSwitchWasActive) {
        missionDebugPrintf("MISSION switch off");
    }

    if (!missionSwitchActive) {
        controlActive = false;
    } else {
        const bool heartbeatFresh = heartbeatSeenSinceEnable
            && cmpTimeUs(currentTimeUs, lastHeartbeatUs) <= MISSION_HEARTBEAT_TIMEOUT_US;
        const bool setpointFresh = setpointSeenSinceEnable
            && cmpTimeUs(currentTimeUs, lastSetpointUs) <= MISSION_SETPOINT_TIMEOUT_US;
        const bool headingFresh = headingSeenSinceEnable
            && cmpTimeUs(currentTimeUs, lastHeadingUs) <= MISSION_HEADING_TIMEOUT_US;

        controlActive = heartbeatFresh && setpointFresh && headingFresh;
        if (!controlActive) {
            missionComputerFailsafe();
        }
    }

    if (controlActive != wasControlActive) {
        missionDebugPrintf("MISSION control=%u", controlActive);
    }

    missionSwitchWasActive = missionSwitchActive;
}

bool missionControlActive(void)
{
    return controlActive;
}

float missionControlAngleDeg(int axis)
{
    if (axis == FD_ROLL) {
        return setpoint.rollCdeg * 0.01f;
    }
    if (axis == FD_PITCH) {
        return setpoint.pitchCdeg * 0.01f;
    }
    return 0.0f;
}

float missionControlYawRateDps(void)
{
    const float gyroDeltaDeg = wrapHeadingErrorDeg((attitude.values.yaw - heading.localYawDeciDeg) * 0.1f);
    const float currentHeadingDeg = heading.headingCdeg * 0.01f + gyroDeltaDeg;
    const float targetHeadingDeg = setpoint.targetYawCdeg * 0.01f;
    const float errorDeg = wrapHeadingErrorDeg(currentHeadingDeg - targetHeadingDeg);
    return constrainf(errorDeg * MISSION_YAW_P, -MISSION_MAX_YAW_RATE_DPS, MISSION_MAX_YAW_RATE_DPS);
}

float missionControlAltitudeCm(void)
{
    return armAltitudeCm + setpoint.altitudeCm;
}

#endif
