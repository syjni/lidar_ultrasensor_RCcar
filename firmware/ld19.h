/*
 * ld19.h
 *
 * Created: 2023-12-30 7:13:40 PM
 *  Author: suhtw
 */ 


#ifndef LD19_H_
#define LD19_H_

enum {
	PKG_HEADER = 0x54,
	PKG_VER_LEN = 0x2C,
	POINT_PER_PACK = 12,
};

#define   packet_len  11 + 3*12    // 47 bytes

typedef struct __attribute__((packed)) {
	uint16_t distance;
	uint8_t intensity;
} LidarPointStructDef;

typedef struct __attribute__((packed)) {
	uint8_t header;
	uint8_t ver_len;
	uint16_t speed;
	uint16_t start_angle;
	LidarPointStructDef point[POINT_PER_PACK];
	uint16_t end_angle;
	uint16_t timestamp;
	uint8_t crc8;
} LiDARFrameTypeDef;


#endif /* LD19_H_ */