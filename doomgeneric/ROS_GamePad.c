/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  File name: ROS_GamePad.c
  Source: /home/joonas/repos/personal/DDS-Doom/doomgeneric/ROS_GamePad.idl
  Cyclone DDS: V0.10.5

*****************************************************************/
#include "ROS_GamePad.h"

static const uint32_t sensor_msgs_msg_GamePad_ops [] =
{
  /* GamePad */
  DDS_OP_ADR | DDS_OP_TYPE_EXT, offsetof (sensor_msgs_msg_GamePad, header), (3u << 16u) + 44u /* Header_ */,
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (sensor_msgs_msg_GamePad, LS_X),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (sensor_msgs_msg_GamePad, LS_Y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (sensor_msgs_msg_GamePad, RS_X),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (sensor_msgs_msg_GamePad, RS_Y),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (sensor_msgs_msg_GamePad, LT),
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_FP, offsetof (sensor_msgs_msg_GamePad, RT),
  DDS_OP_ADR | DDS_OP_TYPE_BLN, offsetof (sensor_msgs_msg_GamePad, start),
  DDS_OP_ADR | DDS_OP_TYPE_BLN, offsetof (sensor_msgs_msg_GamePad, select),
  DDS_OP_ADR | DDS_OP_TYPE_BLN, offsetof (sensor_msgs_msg_GamePad, x),
  DDS_OP_ADR | DDS_OP_TYPE_BLN, offsetof (sensor_msgs_msg_GamePad, y),
  DDS_OP_ADR | DDS_OP_TYPE_BLN, offsetof (sensor_msgs_msg_GamePad, a),
  DDS_OP_ADR | DDS_OP_TYPE_BLN, offsetof (sensor_msgs_msg_GamePad, b),
  DDS_OP_ADR | DDS_OP_TYPE_BLN, offsetof (sensor_msgs_msg_GamePad, dpad_up),
  DDS_OP_ADR | DDS_OP_TYPE_BLN, offsetof (sensor_msgs_msg_GamePad, dpad_down),
  DDS_OP_ADR | DDS_OP_TYPE_BLN, offsetof (sensor_msgs_msg_GamePad, dpad_left),
  DDS_OP_ADR | DDS_OP_TYPE_BLN, offsetof (sensor_msgs_msg_GamePad, dpad_right),
  DDS_OP_ADR | DDS_OP_TYPE_BLN, offsetof (sensor_msgs_msg_GamePad, LB),
  DDS_OP_ADR | DDS_OP_TYPE_BLN, offsetof (sensor_msgs_msg_GamePad, RB),
  DDS_OP_ADR | DDS_OP_TYPE_BLN, offsetof (sensor_msgs_msg_GamePad, LS),
  DDS_OP_ADR | DDS_OP_TYPE_BLN, offsetof (sensor_msgs_msg_GamePad, RS),
  DDS_OP_RTS,

  /* Header_ */
  DDS_OP_ADR | DDS_OP_TYPE_EXT, offsetof (std_msgs_msg_dds__Header_, stamp), (3u << 16u) + 6u /* Time_ */,
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (std_msgs_msg_dds__Header_, frame_id),
  DDS_OP_RTS,

  /* Time_ */
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (std_msgs_msg_dds__Time_, sec),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (std_msgs_msg_dds__Time_, nanosec),
  DDS_OP_RTS
};

/* Type Information:
  [MINIMAL 2b48ffa1dc023cb1cdc738306666] (#deps: 2)
   - [MINIMAL dcf12cd2dd5e712cb7b1e51fa3f2]
   - [MINIMAL 567c5a93541c3b1086a4ba46f98d]
  [COMPLETE f9b42577756b8e71952961dd74a4] (#deps: 2)
   - [COMPLETE b7641a3e9e06ad1f6434bc7f626c]
   - [COMPLETE a68dade0c2af90a0e260105d2ab4]
*/
#define TYPE_INFO_CDR_sensor_msgs_msg_GamePad (unsigned char []){ \
  0xc0, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00, 0x40, 0x58, 0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0x00, \
  0x14, 0x00, 0x00, 0x00, 0xf1, 0x2b, 0x48, 0xff, 0xa1, 0xdc, 0x02, 0x3c, 0xb1, 0xcd, 0xc7, 0x38, \
  0x30, 0x66, 0x66, 0x00, 0x77, 0x01, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x34, 0x00, 0x00, 0x00, \
  0x02, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0xf1, 0xdc, 0xf1, 0x2c, 0xd2, 0xdd, 0x5e, 0x71, \
  0x2c, 0xb7, 0xb1, 0xe5, 0x1f, 0xa3, 0xf2, 0x00, 0x48, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, \
  0xf1, 0x56, 0x7c, 0x5a, 0x93, 0x54, 0x1c, 0x3b, 0x10, 0x86, 0xa4, 0xba, 0x46, 0xf9, 0x8d, 0x00, \
  0x37, 0x00, 0x00, 0x00, 0x02, 0x10, 0x00, 0x40, 0x58, 0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0x00, \
  0x14, 0x00, 0x00, 0x00, 0xf2, 0xf9, 0xb4, 0x25, 0x77, 0x75, 0x6b, 0x8e, 0x71, 0x95, 0x29, 0x61, \
  0xdd, 0x74, 0xa4, 0x00, 0x49, 0x02, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x34, 0x00, 0x00, 0x00, \
  0x02, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0xf2, 0xb7, 0x64, 0x1a, 0x3e, 0x9e, 0x06, 0xad, \
  0x1f, 0x64, 0x34, 0xbc, 0x7f, 0x62, 0x6c, 0x00, 0x7f, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, \
  0xf2, 0xa6, 0x8d, 0xad, 0xe0, 0xc2, 0xaf, 0x90, 0xa0, 0xe2, 0x60, 0x10, 0x5d, 0x2a, 0xb4, 0x00, \
  0x6a, 0x00, 0x00, 0x00\
}
#define TYPE_INFO_CDR_SZ_sensor_msgs_msg_GamePad 196u
#define TYPE_MAP_CDR_sensor_msgs_msg_GamePad (unsigned char []){ \
  0x2b, 0x02, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0xf1, 0x2b, 0x48, 0xff, 0xa1, 0xdc, 0x02, 0x3c, \
  0xb1, 0xcd, 0xc7, 0x38, 0x30, 0x66, 0x66, 0x00, 0x73, 0x01, 0x00, 0x00, 0xf1, 0x51, 0x01, 0x00, \
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x63, 0x01, 0x00, 0x00, 0x15, 0x00, 0x00, 0x00, \
  0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0xf1, 0xdc, 0xf1, 0x2c, 0xd2, 0xdd, \
  0x5e, 0x71, 0x2c, 0xb7, 0xb1, 0xe5, 0x1f, 0xa3, 0xf2, 0x09, 0x9f, 0xb9, 0x95, 0x00, 0x00, 0x00, \
  0x0b, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x09, 0xec, 0x5c, 0x05, 0x87, 0x00, \
  0x0b, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x09, 0x84, 0xbc, 0x40, 0xf2, 0x00, \
  0x0b, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x09, 0xb3, 0xb1, 0xb0, 0x47, 0x00, \
  0x0b, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x09, 0xae, 0x0a, 0x48, 0x34, 0x00, \
  0x0b, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x01, 0x00, 0x09, 0xc5, 0x62, 0x60, 0x71, 0x00, \
  0x0b, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x01, 0x00, 0x09, 0x70, 0x56, 0x10, 0xed, 0x00, \
  0x0b, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0xea, 0x2b, 0x26, 0x76, 0x00, \
  0x0b, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x99, 0x93, 0x82, 0x82, 0x00, \
  0x0b, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x9d, 0xd4, 0xe4, 0x61, 0x00, \
  0x0b, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x41, 0x52, 0x90, 0x76, 0x00, \
  0x0b, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x0c, 0xc1, 0x75, 0xb9, 0x00, \
  0x0b, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x92, 0xeb, 0x5f, 0xfe, 0x00, \
  0x0b, 0x00, 0x00, 0x00, 0x0d, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x1f, 0xe4, 0x72, 0x5e, 0x00, \
  0x0b, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x04, 0x3e, 0x50, 0x61, 0x00, \
  0x0b, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x4c, 0x67, 0x0f, 0x50, 0x00, \
  0x0b, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0xb0, 0xc0, 0x48, 0xa3, 0x00, \
  0x0b, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0xc9, 0x51, 0x27, 0x0e, 0x00, \
  0x0b, 0x00, 0x00, 0x00, 0x12, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0xc8, 0xee, 0x22, 0xe1, 0x00, \
  0x0b, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0xe8, 0x28, 0x09, 0x07, 0x00, \
  0x0b, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x8c, 0xee, 0x50, 0x50, 0xf1, \
  0xdc, 0xf1, 0x2c, 0xd2, 0xdd, 0x5e, 0x71, 0x2c, 0xb7, 0xb1, 0xe5, 0x1f, 0xa3, 0xf2, 0x00, 0x00, \
  0x44, 0x00, 0x00, 0x00, 0xf1, 0x51, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
  0x34, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
  0x01, 0x00, 0xf1, 0x56, 0x7c, 0x5a, 0x93, 0x54, 0x1c, 0x3b, 0x10, 0x86, 0xa4, 0xba, 0x46, 0xf9, \
  0x8d, 0x96, 0xb8, 0xc7, 0x8d, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, \
  0x01, 0x00, 0x70, 0x00, 0x4b, 0xb3, 0x9c, 0x5c, 0xf1, 0x56, 0x7c, 0x5a, 0x93, 0x54, 0x1c, 0x3b, \
  0x10, 0x86, 0xa4, 0xba, 0x46, 0xf9, 0x8d, 0x00, 0x33, 0x00, 0x00, 0x00, 0xf1, 0x51, 0x01, 0x00, \
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, \
  0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x04, 0x74, 0x45, 0x9c, 0xa3, 0x00, \
  0x0b, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x07, 0xe2, 0x04, 0x64, 0xd5, 0x00, \
  0x66, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0xf2, 0xf9, 0xb4, 0x25, 0x77, 0x75, 0x6b, 0x8e, \
  0x71, 0x95, 0x29, 0x61, 0xdd, 0x74, 0xa4, 0x00, 0x45, 0x02, 0x00, 0x00, 0xf2, 0x51, 0x01, 0x00, \
  0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x00, 0x00, 0x00, 0x73, 0x65, 0x6e, 0x73, \
  0x6f, 0x72, 0x5f, 0x6d, 0x73, 0x67, 0x73, 0x3a, 0x3a, 0x6d, 0x73, 0x67, 0x3a, 0x3a, 0x47, 0x61, \
  0x6d, 0x65, 0x50, 0x61, 0x64, 0x00, 0x00, 0x00, 0x15, 0x02, 0x00, 0x00, 0x15, 0x00, 0x00, 0x00, \
  0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0xf2, 0xb7, 0x64, 0x1a, 0x3e, 0x9e, \
  0x06, 0xad, 0x1f, 0x64, 0x34, 0xbc, 0x7f, 0x62, 0x6c, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, \
  0x68, 0x65, 0x61, 0x64, 0x65, 0x72, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, \
  0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x09, 0x00, 0x05, 0x00, 0x00, 0x00, 0x4c, 0x53, 0x5f, 0x58, \
  0x00, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x09, 0x00, \
  0x05, 0x00, 0x00, 0x00, 0x4c, 0x53, 0x5f, 0x59, 0x00, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, \
  0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x09, 0x00, 0x05, 0x00, 0x00, 0x00, 0x52, 0x53, 0x5f, 0x58, \
  0x00, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x09, 0x00, \
  0x05, 0x00, 0x00, 0x00, 0x52, 0x53, 0x5f, 0x59, 0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, \
  0x05, 0x00, 0x00, 0x00, 0x01, 0x00, 0x09, 0x00, 0x03, 0x00, 0x00, 0x00, 0x4c, 0x54, 0x00, 0x00, \
  0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x01, 0x00, 0x09, 0x00, \
  0x03, 0x00, 0x00, 0x00, 0x52, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, \
  0x07, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x06, 0x00, 0x00, 0x00, 0x73, 0x74, 0x61, 0x72, \
  0x74, 0x00, 0x00, 0x00, 0x15, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, \
  0x07, 0x00, 0x00, 0x00, 0x73, 0x65, 0x6c, 0x65, 0x63, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
  0x10, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, \
  0x78, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, \
  0x02, 0x00, 0x00, 0x00, 0x79, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00, \
  0x01, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x61, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, \
  0x0c, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x62, 0x00, 0x00, 0x00, \
  0x16, 0x00, 0x00, 0x00, 0x0d, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x08, 0x00, 0x00, 0x00, \
  0x64, 0x70, 0x61, 0x64, 0x5f, 0x75, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, \
  0x0e, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x64, 0x70, 0x61, 0x64, \
  0x5f, 0x64, 0x6f, 0x77, 0x6e, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, \
  0x01, 0x00, 0x01, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x64, 0x70, 0x61, 0x64, 0x5f, 0x6c, 0x65, 0x66, \
  0x74, 0x00, 0x00, 0x00, 0x19, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, \
  0x0b, 0x00, 0x00, 0x00, 0x64, 0x70, 0x61, 0x64, 0x5f, 0x72, 0x69, 0x67, 0x68, 0x74, 0x00, 0x00, \
  0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, \
  0x03, 0x00, 0x00, 0x00, 0x4c, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, \
  0x12, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x03, 0x00, 0x00, 0x00, 0x52, 0x42, 0x00, 0x00, \
  0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, \
  0x03, 0x00, 0x00, 0x00, 0x4c, 0x53, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, \
  0x14, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x03, 0x00, 0x00, 0x00, 0x52, 0x53, 0x00, 0x00, \
  0x00, 0xf2, 0xb7, 0x64, 0x1a, 0x3e, 0x9e, 0x06, 0xad, 0x1f, 0x64, 0x34, 0xbc, 0x7f, 0x62, 0x6c, \
  0x7b, 0x00, 0x00, 0x00, 0xf2, 0x51, 0x01, 0x00, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
  0x1d, 0x00, 0x00, 0x00, 0x73, 0x74, 0x64, 0x5f, 0x6d, 0x73, 0x67, 0x73, 0x3a, 0x3a, 0x6d, 0x73, \
  0x67, 0x3a, 0x3a, 0x64, 0x64, 0x73, 0x5f, 0x3a, 0x3a, 0x48, 0x65, 0x61, 0x64, 0x65, 0x72, 0x5f, \
  0x00, 0x00, 0x00, 0x00, 0x47, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, \
  0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0xf2, 0xa6, 0x8d, 0xad, 0xe0, 0xc2, 0xaf, 0x90, 0xa0, 0xe2, \
  0x60, 0x10, 0x5d, 0x2a, 0xb4, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x73, 0x74, 0x61, 0x6d, \
  0x70, 0x00, 0x00, 0x00, 0x17, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x70, 0x00, \
  0x09, 0x00, 0x00, 0x00, 0x66, 0x72, 0x61, 0x6d, 0x65, 0x5f, 0x69, 0x64, 0x00, 0x00, 0x00, 0xf2, \
  0xa6, 0x8d, 0xad, 0xe0, 0xc2, 0xaf, 0x90, 0xa0, 0xe2, 0x60, 0x10, 0x5d, 0x2a, 0xb4, 0x00, 0x00, \
  0x66, 0x00, 0x00, 0x00, 0xf2, 0x51, 0x01, 0x00, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
  0x1b, 0x00, 0x00, 0x00, 0x73, 0x74, 0x64, 0x5f, 0x6d, 0x73, 0x67, 0x73, 0x3a, 0x3a, 0x6d, 0x73, \
  0x67, 0x3a, 0x3a, 0x64, 0x64, 0x73, 0x5f, 0x3a, 0x3a, 0x54, 0x69, 0x6d, 0x65, 0x5f, 0x00, 0x00, \
  0x36, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
  0x01, 0x00, 0x04, 0x00, 0x04, 0x00, 0x00, 0x00, 0x73, 0x65, 0x63, 0x00, 0x00, 0x00, 0x00, 0x00, \
  0x16, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x07, 0x00, 0x08, 0x00, 0x00, 0x00, \
  0x6e, 0x61, 0x6e, 0x6f, 0x73, 0x65, 0x63, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5e, 0x00, 0x00, 0x00, \
  0x03, 0x00, 0x00, 0x00, 0xf2, 0xf9, 0xb4, 0x25, 0x77, 0x75, 0x6b, 0x8e, 0x71, 0x95, 0x29, 0x61, \
  0xdd, 0x74, 0xa4, 0xf1, 0x2b, 0x48, 0xff, 0xa1, 0xdc, 0x02, 0x3c, 0xb1, 0xcd, 0xc7, 0x38, 0x30, \
  0x66, 0x66, 0xf2, 0xb7, 0x64, 0x1a, 0x3e, 0x9e, 0x06, 0xad, 0x1f, 0x64, 0x34, 0xbc, 0x7f, 0x62, \
  0x6c, 0xf1, 0xdc, 0xf1, 0x2c, 0xd2, 0xdd, 0x5e, 0x71, 0x2c, 0xb7, 0xb1, 0xe5, 0x1f, 0xa3, 0xf2, \
  0xf2, 0xa6, 0x8d, 0xad, 0xe0, 0xc2, 0xaf, 0x90, 0xa0, 0xe2, 0x60, 0x10, 0x5d, 0x2a, 0xb4, 0xf1, \
  0x56, 0x7c, 0x5a, 0x93, 0x54, 0x1c, 0x3b, 0x10, 0x86, 0xa4, 0xba, 0x46, 0xf9, 0x8d\
}
#define TYPE_MAP_CDR_SZ_sensor_msgs_msg_GamePad 1534u
const dds_topic_descriptor_t sensor_msgs_msg_GamePad_desc =
{
  .m_size = sizeof (sensor_msgs_msg_GamePad),
  .m_align = dds_alignof (sensor_msgs_msg_GamePad),
  .m_flagset = DDS_TOPIC_XTYPES_METADATA,
  .m_nkeys = 0u,
  .m_typename = "sensor_msgs::msg::GamePad",
  .m_keys = NULL,
  .m_nops = 28,
  .m_ops = sensor_msgs_msg_GamePad_ops,
  .m_meta = "",
  .type_information = { .data = TYPE_INFO_CDR_sensor_msgs_msg_GamePad, .sz = TYPE_INFO_CDR_SZ_sensor_msgs_msg_GamePad },
  .type_mapping = { .data = TYPE_MAP_CDR_sensor_msgs_msg_GamePad, .sz = TYPE_MAP_CDR_SZ_sensor_msgs_msg_GamePad }
};
