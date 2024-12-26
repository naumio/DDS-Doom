/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  File name: ROS_Header.h
  Source: /home/joonas/repos/personal/DDS-Doom/doomgeneric/ROS_Header.idl
  Cyclone DDS: V0.10.5

*****************************************************************/
#ifndef DDSC_ROS_HEADER_H
#define DDSC_ROS_HEADER_H

#include "ROS_Time.h"

#include "dds/ddsc/dds_public_impl.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct std_msgs_msg_dds__Header_
{
  struct std_msgs_msg_dds__Time_ stamp;
  char * frame_id;
} std_msgs_msg_dds__Header_;

extern const dds_topic_descriptor_t std_msgs_msg_dds__Header__desc;

#define std_msgs_msg_dds__Header___alloc() \
((std_msgs_msg_dds__Header_*) dds_alloc (sizeof (std_msgs_msg_dds__Header_)));

#define std_msgs_msg_dds__Header__free(d,o) \
dds_sample_free ((d), &std_msgs_msg_dds__Header__desc, (o))

#ifdef __cplusplus
}
#endif

#endif /* DDSC_ROS_HEADER_H */
