#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <memory.h>
#include "protocol_ibus.h"

#define IBUS_2BYTE_SESNSOR          2
#define IBUS_4BYTE_SESNSOR          4

uint8_t flysky_sensors[IBUS_SENSOR_COUNT] = {
    IBUS_SENSOR_TYPE_NONE,
    IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE,
    IBUS_SENSOR_TYPE_TEMPERATURE,
    IBUS_SENSOR_TYPE_CELL
};

uint8_t ibusIsValidIa6bIbusPacketLength(uint8_t length)
{
    return (length == IBUS_TELEMETRY_PACKET_LENGTH) || (length == IBUS_SERIAL_RX_PACKET_LENGTH);
}

uint16_t ibusCalculateChecksum(const uint8_t *ibusPacket)
{
    uint16_t checksum = 0xFFFF;
    uint8_t dataSize = ibusPacket[0] - IBUS_CHECKSUM_SIZE;
    for (unsigned i = 0; i < dataSize; i++) {
        checksum -= ibusPacket[i];
    }

    return checksum;
}

uint8_t ibusIsChecksumOkIa6(ibus_t* ibus)
{
    uint8_t offset;
    uint8_t i;
    uint16_t chksum, rxsum;
    chksum = ibus->ibusChecksum;
    rxsum = ibus->recvBuffer[ibus->ibusFrameSize - 2] + (ibus->recvBuffer[ibus->ibusFrameSize - 1] << 8);
    for (i = 0, offset = ibus->ibusChannelOffset; i < IBUS_MAX_SLOTS; i++, offset += 2) {
        chksum += ibus->recvBuffer[offset] + (ibus->recvBuffer[offset + 1] << 8);
    }
    return chksum == rxsum;
}

bool ibusIsChecksumOkIa6b(const uint8_t *ibusPacket, const uint8_t length)
{
    uint16_t calculatedChecksum = ibusCalculateChecksum(ibusPacket);

    // Note that there's a byte order swap to little endian here
    return (calculatedChecksum >> 8) == ibusPacket[length - 1]
           && (calculatedChecksum & 0xFF) == ibusPacket[length - 2];
}

uint8_t ibusChecksumIsOk(ibus_t* ibus)
{
    if (ibus->ibusModel == IBUS_MODEL_IA6 ) {
        return ibusIsChecksumOkIa6(ibus);
    } else {
        return ibusIsChecksumOkIa6b(ibus->recvBuffer, ibus->ibusFrameSize);
    }
}

void ibusUpdateChannelData(ibus_t* ibus)
{
    uint8_t i;
    uint8_t offset;
    for (i = 0, offset = ibus->ibusChannelOffset; i < IBUS_MAX_SLOTS; i++, offset += 2) {
        ibus->ibusChannelData[i] = ibus->recvBuffer[offset] + ((ibus->recvBuffer[offset + 1] & 0x0F) << 8);
    }
    //latest IBUS recievers are using prviously not used 4 bits on every channel to increase total channel count
    for (i = IBUS_MAX_SLOTS, offset = ibus->ibusChannelOffset + 1; i < IBUS_MAX_CHANNEL; i++, offset += 6) {
        ibus->ibusChannelData[i] = ((ibus->recvBuffer[offset] & 0xF0) >> 4) | (ibus->recvBuffer[offset + 2] & 0xF0) | ((ibus->recvBuffer[offset + 4] & 0xF0) << 4);
    }
}

uint8_t getSensorID(uint8_t address)
{
    return flysky_sensors[address];
}

uint8_t getSensorLength(uint8_t sensorType)
{
    if (sensorType == IBUS_SENSOR_TYPE_PRES || (sensorType >= IBUS_SENSOR_TYPE_GPS_LAT && sensorType <= IBUS_SENSOR_TYPE_ALT_MAX)) {
            return IBUS_4BYTE_SESNSOR;
        }
    #if defined(USE_TELEMETRY_IBUS_EXTENDED)
        uint8_t itemCount;
        const uint8_t* structure = getSensorStruct(sensorType, &itemCount);
        if (structure != 0) {
            uint8_t size = 0;
            for (unsigned i = 0; i < itemCount; i++) {
                size += getSensorLength(structure[i]);
            }
            return size;
        }
    #endif //defined(USE_TELEMETRY_IBUS_EXTENDED)
        return IBUS_2BYTE_SESNSOR;
}

void ibusSetDiscoverSensor(ibus_t* ibus, uint8_t address)
{
    ibus->sendBuffer[0] = IBUS_HEADER_FOOTER_SIZE;
    ibus->sendBuffer[1] = IBUS_COMMAND_DISCOVER_SENSOR | address;
    
    uint16_t checksum = ibusCalculateChecksum(ibus->sendBuffer);
    ibus->sendBuffer[2] = checksum & 0xFF;
    ibus->sendBuffer[3] = checksum >> 8;

    // Send command
    ibus->writeData(ibus->sendBuffer, IBUS_HEADER_FOOTER_SIZE);
}

void ibusSetSensorType(ibus_t* ibus, uint8_t address)
{
    uint8_t sensorID = getSensorID(address);
    uint8_t sensorLength = getSensorLength(sensorID);
    ibus->sendBuffer[0] = IBUS_HEADER_FOOTER_SIZE + 2;
    ibus->sendBuffer[1] = IBUS_COMMAND_SENSOR_TYPE | address;
    ibus->sendBuffer[2] = sensorID;
    ibus->sendBuffer[3] = sensorLength;

    uint16_t checksum = ibusCalculateChecksum(ibus->sendBuffer);
    ibus->sendBuffer[4] = checksum & 0xFF;
    ibus->sendBuffer[5] = checksum >> 8;

    // Send command
    ibus->writeData(ibus->sendBuffer, IBUS_HEADER_FOOTER_SIZE + 2);
}

void ibusSetMeasurement(ibus_t* ibus, uint8_t address)
{
    ibusTelemetry_s value;
    uint8_t sensorID = getSensorID(address);
    uint8_t sensorLength = getSensorLength(sensorID);
    uint8_t packetSize = IBUS_HEADER_FOOTER_SIZE + sensorLength;
    ibus->sendBuffer[0] = packetSize;
    ibus->sendBuffer[1] = IBUS_COMMAND_MEASUREMENT | address;

#if defined(USE_TELEMETRY_IBUS_EXTENDED)
    uint8_t itemCount;
    const uint8_t* structure = getSensorStruct(sensorType, &itemCount);
    if (structure != 0) {
        setCombinedFrame(bufferPtr, structure, itemCount);
        return;
    }
#endif //defined(USE_TELEMETRY_IBUS_EXTENDED)
    //clear result
    for (unsigned i = 2; i < sensorLength; i++) {
        ibus->sendBuffer[i] = value.byte[i] = 0;
    }
#if defined(USE_GPS)
    if (setGPS(sensorType, &value)) {
        for (unsigned i = 0; i < length; i++) {
            bufferPtr[i] = value.byte[i];
        }
        return;
    }
#endif //defined(USE_TELEMETRY_IBUS_EXTENDED)
    switch (sensorID) {
        case IBUS_SENSOR_TYPE_NONE:
            value.uint16 = 2;
        break;
        case IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE:
            value.uint16 = 1; //getVoltage();
            break;
        case IBUS_SENSOR_TYPE_TEMPERATURE:
            value.uint16 = 2; //getTemperature();
            break;
        case IBUS_SENSOR_TYPE_RPM_FLYSKY:
            value.int16 = 3; //(int16_t)rcCommand[THROTTLE];
            break;
        case IBUS_SENSOR_TYPE_FUEL:
            value.uint16 = 4; //getFuel();
            break;
        case IBUS_SENSOR_TYPE_RPM:
            value.uint16 = 5; //getRPM();
            break;
        case IBUS_SENSOR_TYPE_FLIGHT_MODE:
            value.uint16 = 6; //getMode();
            break;
        case IBUS_SENSOR_TYPE_CELL:
            value.uint16 = 7; //(uint16_t)(getBatteryAverageCellVoltage());
            break;
        case IBUS_SENSOR_TYPE_BAT_CURR:
            value.uint16 = 8; //(uint16_t)getAmperage();
            break;
#if defined(USE_ACC)
        case IBUS_SENSOR_TYPE_ACC_X:
        case IBUS_SENSOR_TYPE_ACC_Y:
        case IBUS_SENSOR_TYPE_ACC_Z:
            value.int16 = 9; //getACC(sensorType - IBUS_SENSOR_TYPE_ACC_X);
            break;
#endif
        case IBUS_SENSOR_TYPE_ROLL:
        case IBUS_SENSOR_TYPE_PITCH:
        case IBUS_SENSOR_TYPE_YAW:
            value.int16 = 10; //attitude.raw[sensorType - IBUS_SENSOR_TYPE_ROLL] *10;
            break;
        case IBUS_SENSOR_TYPE_ARMED:
            value.uint16 = 11; //ARMING_FLAG(ARMED) ? 1 : 0;
            break;
#if defined(USE_TELEMETRY_IBUS_EXTENDED)
        case IBUS_SENSOR_TYPE_CMP_HEAD:
            value.uint16 = 12; //DECIDEGREES_TO_DEGREES(attitude.values.yaw);
            break;
#ifdef USE_VARIO
        case IBUS_SENSOR_TYPE_VERTICAL_SPEED:
        case IBUS_SENSOR_TYPE_CLIMB_RATE:
            value.int16 = 13; //(int16_t) constrain(getEstimatedVario(), SHRT_MIN, SHRT_MAX);
            break;
#endif
#ifdef USE_BARO
        case IBUS_SENSOR_TYPE_ALT:
        case IBUS_SENSOR_TYPE_ALT_MAX:
            value.int32 = 14; //baro.altitude;
            break;
        case IBUS_SENSOR_TYPE_PRES:
            value.uint32 = 15; //baro.pressure | (((uint32_t)getTemperature()) << 19);
            break;
#endif
#endif //defined(TELEMETRY_IBUS_EXTENDED)
    }

    // Fill in the bytes
    unsigned i = 2;
    for (; i < sensorLength; i++) {
        ibus->sendBuffer[i] = value.byte[i];
    }

    uint16_t checksum = ibusCalculateChecksum(ibus->sendBuffer);
    ibus->sendBuffer[i] = checksum & 0xFF;
    ibus->sendBuffer[i + 1] = checksum >> 8;

     // Send command
    ibus->writeData(ibus->sendBuffer, packetSize);
}

// Receive ISR callback
void ibusProcessDataISR(ibus_t* ibus, uint8_t c, uint32_t timeInMicroSeconds)
{
    static uint32_t ibusTimeLast;
    static uint8_t ibusFramePosition;
    
    int d = timeInMicroSeconds - ibusTimeLast;
    if (d > IBUS_FRAME_GAP) {
       ibusFramePosition = 0;
       ibus->rxBytesToIgnore = 0;
    } else if (ibus->rxBytesToIgnore) {
        ibus->rxBytesToIgnore--;
        return;
    }

    ibusTimeLast = timeInMicroSeconds;

    if (ibusFramePosition == 0) {
        if (ibusIsValidIa6bIbusPacketLength(c)) {
            ibus->ibusModel = IBUS_MODEL_IA6B;
            ibus->ibusSyncByte = c;
            ibus->ibusFrameSize = c;
            ibus->ibusChannelOffset = 2;
            ibus->ibusChecksum = 0xFFFF;
        } else if ((ibus->ibusSyncByte == 0) && (c == 0x55)) {
            ibus->ibusModel = IBUS_MODEL_IA6;
            ibus->ibusSyncByte = 0x55;
            ibus->ibusFrameSize = 31;
            ibus->ibusChecksum = 0x0000;
            ibus->ibusChannelOffset = 1;
        } else if (ibus->ibusSyncByte != c) {
            return;
        }
    }

    ibus->recvBuffer[ibusFramePosition] = (uint8_t)c;

    if (ibusFramePosition == ibus->ibusFrameSize - 1) {
        ibus->lastFrameTimeUs = timeInMicroSeconds;
        ibus->ibusFrameDone = true;
        ibus->receivedData(ibus->recvBuffer, ibus->ibusFrameSize);
    } else {
        ibusFramePosition++;
    }
}

void ibusTelemetry(ibus_t* ibus, uint8_t* data)
{
    if (data[0] != IBUS_TELEMETRY_PACKET_LENGTH)
        return;

    uint8_t sensorType = data[1] & 0xF0;
    uint8_t address = data[1] & 0x0F;
    switch (sensorType)
    {
        case IBUS_COMMAND_DISCOVER_SENSOR:
            ibusSetDiscoverSensor(ibus, address);
        break;
        case IBUS_COMMAND_SENSOR_TYPE:
            ibusSetSensorType(ibus, address);
        break;
        case IBUS_COMMAND_MEASUREMENT:
            ibusSetMeasurement(ibus, address);
        break;
    }
}

/// @brief 
/// @param ibus 
/// @param rxConfig 
/// @return 
bool ibusInit(ibus_t* ibus, const ibusConfig_t *rxConfig)
{
    memset(ibus, 0, sizeof(*ibus));
    ibus->readData = rxConfig->ReadData;
    ibus->writeData = rxConfig->WriteData;
    ibus->receivedData = rxConfig->ReceivedData;
 
    return true;
}