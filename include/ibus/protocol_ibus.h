#ifndef IBUS_H_
#define IBUS_H_

#ifdef __cplusplus
extern "C" {
#endif

#define IBUS_MAX_CHANNEL 18

//In AFHDS there is 18 channels encoded in 14 slots (each slot is 2 byte long)
#define IBUS_MAX_SLOTS 14
#define IBUS_BUFFSIZE 33
#define IBUS_MODEL_IA6B 0
#define IBUS_MODEL_IA6 1
#define IBUS_FRAME_GAP 500
#define IBUS_BAUDRATE 115200
#define IBUS_TELEMETRY_PACKET_LENGTH (4)
#define IBUS_SERIAL_RX_PACKET_LENGTH (32)
#define IBUS_CHECKSUM_SIZE (2)
#define IBUS_HEADER_FOOTER_SIZE     4

// Forward declaration
typedef struct ibusConfig ibusConfig_t;

typedef bool (*ibusReadData_t)(uint8_t* data, uint32_t size);
typedef bool (*ibusWriteData_t)(uint8_t* data, uint32_t size);
typedef bool (*ibusDataReceived_t)(uint8_t* data, uint32_t size);

typedef enum {
    IBUS_COMMAND_DISCOVER_SENSOR      = 0x80,
    IBUS_COMMAND_SENSOR_TYPE          = 0x90,
    IBUS_COMMAND_MEASUREMENT          = 0xA0
} ibusCommand_e;

typedef union ibusTelemetry {
    uint16_t uint16;
    uint32_t uint32;
    int16_t int16;
    int32_t int32;
    uint8_t byte[4];
} ibusTelemetry_s;

#define IBUS_CHECKSUM_SIZE (2)
#define IBUS_SENSOR_COUNT 15

typedef enum {
    IBUS_SENSOR_TYPE_NONE             = 0x00,
    IBUS_SENSOR_TYPE_TEMPERATURE      = 0x01,
    IBUS_SENSOR_TYPE_RPM_FLYSKY       = 0x02,
    IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE = 0x03,
    IBUS_SENSOR_TYPE_CELL             = 0x04, // Avg Cell voltage
    IBUS_SENSOR_TYPE_BAT_CURR         = 0x05, // battery current A * 100
    IBUS_SENSOR_TYPE_FUEL             = 0x06, // remaining battery percentage / mah drawn otherwise or fuel level no unit!
    IBUS_SENSOR_TYPE_RPM              = 0x07, // throttle value / battery capacity
    IBUS_SENSOR_TYPE_CMP_HEAD         = 0x08, //Heading  0..360 deg, 0=north 2bytes
    IBUS_SENSOR_TYPE_CLIMB_RATE       = 0x09, //2 bytes m/s *100
    IBUS_SENSOR_TYPE_COG              = 0x0a, //2 bytes  Course over ground(NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. unknown max uint
    IBUS_SENSOR_TYPE_GPS_STATUS       = 0x0b, //2 bytes
    IBUS_SENSOR_TYPE_ACC_X            = 0x0c, //2 bytes m/s *100 signed
    IBUS_SENSOR_TYPE_ACC_Y            = 0x0d, //2 bytes m/s *100 signed
    IBUS_SENSOR_TYPE_ACC_Z            = 0x0e, //2 bytes m/s *100 signed
    IBUS_SENSOR_TYPE_ROLL             = 0x0f, //2 bytes deg *100 signed
    IBUS_SENSOR_TYPE_PITCH            = 0x10, //2 bytes deg *100 signed
    IBUS_SENSOR_TYPE_YAW              = 0x11, //2 bytes deg *100 signed
    IBUS_SENSOR_TYPE_VERTICAL_SPEED   = 0x12, //2 bytes m/s *100
    IBUS_SENSOR_TYPE_GROUND_SPEED     = 0x13, //2 bytes m/s *100 different unit than build-in sensor
    IBUS_SENSOR_TYPE_GPS_DIST         = 0x14, //2 bytes dist from home m unsigned
    IBUS_SENSOR_TYPE_ARMED            = 0x15, //2 bytes
    IBUS_SENSOR_TYPE_FLIGHT_MODE      = 0x16, //2 bytes
    IBUS_SENSOR_TYPE_PRES             = 0x41, // Pressure
    IBUS_SENSOR_TYPE_ODO1             = 0x7c, // Odometer1
    IBUS_SENSOR_TYPE_ODO2             = 0x7d, // Odometer2
    IBUS_SENSOR_TYPE_SPE              = 0x7e, // Speed 2bytes km/h

    IBUS_SENSOR_TYPE_GPS_LAT          = 0x80, //4bytes signed WGS84 in degrees * 1E7
    IBUS_SENSOR_TYPE_GPS_LON          = 0x81, //4bytes signed WGS84 in degrees * 1E7
    IBUS_SENSOR_TYPE_GPS_ALT          = 0x82, //4bytes signed!!! GPS alt m*100
    IBUS_SENSOR_TYPE_ALT              = 0x83, //4bytes signed!!! Alt m*100
    IBUS_SENSOR_TYPE_ALT_MAX          = 0x84, //4bytes signed MaxAlt m*100

    IBUS_SENSOR_TYPE_ALT_FLYSKY       = 0xf9, // Altitude 2 bytes signed in m
#if defined(USE_TELEMETRY_IBUS_EXTENDED)
    IBUS_SENSOR_TYPE_GPS_FULL         = 0xfd,
    IBUS_SENSOR_TYPE_VOLT_FULL        = 0xf0,
    IBUS_SENSOR_TYPE_ACC_FULL         = 0xef,
#endif //defined(TELEMETRY_IBUS_EXTENDED)
    IBUS_SENSOR_TYPE_UNKNOWN          = 0xff
} ibusSensorType_e;

typedef struct ibus
{
    uint32_t ibusChannelData[IBUS_MAX_CHANNEL];
    uint8_t recvBuffer[32];
    uint8_t sendBuffer[32];
    uint32_t lastFrameTimeUs;
    uint8_t ibusSyncByte;
    uint8_t rxBytesToIgnore;
    uint8_t ibusModel;
    uint8_t ibusFrameSize;
    uint8_t ibusChannelOffset;
    uint16_t ibusChecksum;
    bool ibusFrameDone;
    ibusReadData_t readData;
    ibusWriteData_t writeData;
    ibusDataReceived_t receivedData;
} ibus_t;

typedef struct ibusConfig
{
    ibusReadData_t ReadData;
    ibusWriteData_t WriteData;
    ibusDataReceived_t ReceivedData;
} ibusConfig_t;

bool ibusInit(ibus_t* ibus, const ibusConfig_t *rxConfig);
void ibusTelemetry(ibus_t* ibus, uint8_t* data);
void ibusUpdateChannelData(ibus_t* ibus);
void ibusProcessDataISR(ibus_t* ibus, uint8_t c, uint32_t timeInMicroSeconds);
uint16_t ibusCalculateChecksum(const uint8_t *ibusPacket);
uint8_t ibusIsValidIa6bIbusPacketLength(uint8_t length);

#ifdef __cplusplus
}
#endif
#endif // IBUS_H_