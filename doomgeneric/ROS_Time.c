/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  File name: ROS_Time.c
  Source: /home/joonas/repos/personal/DDS-Doom/doomgeneric/ROS_Time.idl
  Cyclone DDS: V0.10.5

*****************************************************************/
#include "ROS_Time.h"

static const uint32_t std_msgs_msg_dds__Time__ops [] =
{
  /* Time_ */
  DDS_OP_ADR | DDS_OP_TYPE_4BY | DDS_OP_FLAG_SGN, offsetof (std_msgs_msg_dds__Time_, sec),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (std_msgs_msg_dds__Time_, nanosec),
  DDS_OP_RTS
};

/* Type Information:
  [MINIMAL 567c5a93541c3b1086a4ba46f98d] (#deps: 0)
  [COMPLETE a68dade0c2af90a0e260105d2ab4] (#deps: 0)
*/
#define TYPE_INFO_CDR_std_msgs_msg_dds__Time_ (unsigned char []){ \
  0x60, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00, 0x40, 0x28, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, \
  0x14, 0x00, 0x00, 0x00, 0xf1, 0x56, 0x7c, 0x5a, 0x93, 0x54, 0x1c, 0x3b, 0x10, 0x86, 0xa4, 0xba, \
  0x46, 0xf9, 0x8d, 0x00, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, \
  0x00, 0x00, 0x00, 0x00, 0x02, 0x10, 0x00, 0x40, 0x28, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, \
  0x14, 0x00, 0x00, 0x00, 0xf2, 0xa6, 0x8d, 0xad, 0xe0, 0xc2, 0xaf, 0x90, 0xa0, 0xe2, 0x60, 0x10, \
  0x5d, 0x2a, 0xb4, 0x00, 0x6a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, \
  0x00, 0x00, 0x00, 0x00\
}
#define TYPE_INFO_CDR_SZ_std_msgs_msg_dds__Time_ 100u
#define TYPE_MAP_CDR_std_msgs_msg_dds__Time_ (unsigned char []){ \
  0x4b, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xf1, 0x56, 0x7c, 0x5a, 0x93, 0x54, 0x1c, 0x3b, \
  0x10, 0x86, 0xa4, 0xba, 0x46, 0xf9, 0x8d, 0x00, 0x33, 0x00, 0x00, 0x00, 0xf1, 0x51, 0x01, 0x00, \
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, \
  0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x04, 0x74, 0x45, 0x9c, 0xa3, 0x00, \
  0x0b, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x07, 0xe2, 0x04, 0x64, 0xd5, 0x00, \
  0x7e, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xf2, 0xa6, 0x8d, 0xad, 0xe0, 0xc2, 0xaf, 0x90, \
  0xa0, 0xe2, 0x60, 0x10, 0x5d, 0x2a, 0xb4, 0x00, 0x66, 0x00, 0x00, 0x00, 0xf2, 0x51, 0x01, 0x00, \
  0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x00, 0x00, 0x00, 0x73, 0x74, 0x64, 0x5f, \
  0x6d, 0x73, 0x67, 0x73, 0x3a, 0x3a, 0x6d, 0x73, 0x67, 0x3a, 0x3a, 0x64, 0x64, 0x73, 0x5f, 0x3a, \
  0x3a, 0x54, 0x69, 0x6d, 0x65, 0x5f, 0x00, 0x00, 0x36, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, \
  0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x04, 0x00, 0x04, 0x00, 0x00, 0x00, \
  0x73, 0x65, 0x63, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, \
  0x01, 0x00, 0x07, 0x00, 0x08, 0x00, 0x00, 0x00, 0x6e, 0x61, 0x6e, 0x6f, 0x73, 0x65, 0x63, 0x00, \
  0x00, 0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xf2, 0xa6, 0x8d, 0xad, \
  0xe0, 0xc2, 0xaf, 0x90, 0xa0, 0xe2, 0x60, 0x10, 0x5d, 0x2a, 0xb4, 0xf1, 0x56, 0x7c, 0x5a, 0x93, \
  0x54, 0x1c, 0x3b, 0x10, 0x86, 0xa4, 0xba, 0x46, 0xf9, 0x8d\
}
#define TYPE_MAP_CDR_SZ_std_msgs_msg_dds__Time_ 250u
const dds_topic_descriptor_t std_msgs_msg_dds__Time__desc =
{
  .m_size = sizeof (std_msgs_msg_dds__Time_),
  .m_align = dds_alignof (std_msgs_msg_dds__Time_),
  .m_flagset = DDS_TOPIC_FIXED_SIZE | DDS_TOPIC_XTYPES_METADATA,
  .m_nkeys = 0u,
  .m_typename = "std_msgs::msg::dds_::Time_",
  .m_keys = NULL,
  .m_nops = 3,
  .m_ops = std_msgs_msg_dds__Time__ops,
  .m_meta = "",
  .type_information = { .data = TYPE_INFO_CDR_std_msgs_msg_dds__Time_, .sz = TYPE_INFO_CDR_SZ_std_msgs_msg_dds__Time_ },
  .type_mapping = { .data = TYPE_MAP_CDR_std_msgs_msg_dds__Time_, .sz = TYPE_MAP_CDR_SZ_std_msgs_msg_dds__Time_ }
};

