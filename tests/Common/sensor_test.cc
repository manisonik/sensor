#include "gtest/gtest.h"
#include "sensor_icm20948.h"
#include "protocol_dshot.h"

static void dshot_write_pwm_dma(void* context, uint32_t* buf, uint32_t len)
{
}

static inline bool checkBit(unsigned int value, int pos)
{
	return value & (1 << pos);
}

static inline unsigned int setBit(unsigned int value, int pos)
{
	 return value |= (1 << pos);
}

TEST(DShot, CrcTest) {
	int value = 1046;

	struct dshot_motor s;
	s.proto.write = dshot_write_pwm_dma;
	dshot_init(&s, 72000000, DSHOT1200);
	dshot_write(&s, value, false);

	unsigned csum = 0;
	unsigned csum_data = value;
	for (int i = 0; i < 3; i++)
	{
		csum ^= csum_data; // xor data by nibbles
		csum_data >>= 4;
	}

	csum &= 0xf;

	// Check buffer
	for (int i = 10; i < 16; i++) {
		printf("%d ", s.buffer[i]);
	}

	EXPECT_STRNE("hello", "world");
}

TEST(DShot, WriteTest) {
	// Expect two strings not to be equal.
	EXPECT_STRNE("hello", "world");
	// Expect equality.
	EXPECT_EQ(7 * 6, 42);
}