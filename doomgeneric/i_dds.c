//
// Copyright(C) 2005-2014 Simon Howard
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// DESCRIPTION:
//       DDS input code.
//


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <math.h>
#include <time.h>

#include "doomtype.h"
#include "d_event.h"
#include "i_system.h"

#include "m_config.h"
#include "m_misc.h"
#include "r_state.h"

#include "doomgeneric.h"

#include "dds/dds.h"
#include "ROS_GamePad.h"
#include "ROS_Image.h"
#include "ROS_Image_shm.h"
#include "ROS_PoseArray.h"

#define READ_BUF_SIZE 1
#define MAX_NUM_POSES 32
#define GAMEPAD_DDS_SUB_TOPIC_NAME "rt/dds_gamepad"
#define IMAGE_DDS_PUB_TOPIC_NAME "rt/dds_doom_image"
#define IMAGE_SHM_DDS_PUB_TOPIC_NAME "rt/dds_doom_image_shm"
#define OBJ_POSES_DDS_PUB_TOPIC_NAME "rt/obj_poses"

static dds_entity_t participant = -1;

static dds_entity_t topic_input = -1;
static dds_entity_t reader_input = -1;

static dds_entity_t topic_image = -1;
static dds_entity_t writer_image = -1;

static dds_entity_t topic_image_shm = -1;
static dds_entity_t writer_image_shm = -1;

static dds_entity_t topic_obj_poses = -1;
static dds_entity_t writer_obj_poses = -1;

static void *read_buf[READ_BUF_SIZE] = {0};
static dds_sample_info_t sample_infos[READ_BUF_SIZE] = {0};
static sensor_msgs_msg_dds__Image_ image;
static sensor_msgs_msg_dds__Image_shm_ image_shm;
static char *shm_name = "doom_image_shm";
static int shm_fd;
static void *p_shm = NULL;
static geometry_msgs_msg_dds__PoseArray_ pose_array;

static int weapon = 0;

static void dds_destroy();

void I_ShutdownDDS(void)
{
    dds_destroy();
    printf("I_ShutdownDDS\n");
}

/**
 * @brief Convert RPY angles to quaternion
 * 
 * @param q     Target quaternion
 * @param roll  Roll angle in radians 
 * @param pitch Pitch angle in radians
 * @param yaw   Yaw angle in radians
 * @return int  0 = Success, -1 = Error
 */
int RPY2Quat(geometry_msgs_msg_dds__Quaternion_ *q, double roll, double pitch, double yaw)
{
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    q->w = cr * cp * cy + sr * sp * sy;
    q->x = sr * cp * cy - cr * sp * sy;
    q->y = cr * sp * cy + sr * cp * sy;
    q->z = cr * cp * sy - sr * sp * cy;

    return 0;
}

void I_InitDDS(void)
{
    /* Register shudown callback */
    I_AtExit(I_ShutdownDDS, true);

    /* Initialize qos for image topic */
//    dds_qos_t *image_qos = NULL;
    dds_qos_t *image_qos = dds_create_qos ();
    dds_qset_reliability (image_qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS (1));
    dds_qset_durability(image_qos, DDS_DURABILITY_VOLATILE);
    dds_qset_history(image_qos, DDS_HISTORY_KEEP_LAST, 1);
    dds_qset_resource_limits (image_qos, 1, DDS_LENGTH_UNLIMITED, DDS_LENGTH_UNLIMITED);

    /* Initialize participant, topic etc. */
    participant = dds_create_participant (DDS_DOMAIN_DEFAULT, NULL, NULL);
    if (participant < 0)
    {
        printf("I_InitDDS: Unable to create DDS participant: %s\n", dds_strretcode(-participant));
        return;
    }

    topic_input = dds_create_topic (participant, &sensor_msgs_msg_GamePad_desc, GAMEPAD_DDS_SUB_TOPIC_NAME, NULL, NULL);
    if (topic_input < 0)
    {
        printf("I_InitDDSS: Unable to create DDS topic: %s\n", dds_strretcode(-topic_input));
        dds_destroy();
        return;
    }

    topic_image = dds_create_topic (participant, &sensor_msgs_msg_dds__Image__desc, IMAGE_DDS_PUB_TOPIC_NAME, image_qos, NULL);
    if (topic_image < 0)
    {
        printf("I_InitDDSS: Unable to create DDS topic: %s\n", dds_strretcode(-topic_image));
        dds_destroy();
        return;
    }

    topic_image_shm = dds_create_topic (participant, &sensor_msgs_msg_dds__Image_shm__desc, IMAGE_SHM_DDS_PUB_TOPIC_NAME, image_qos, NULL);
    if (topic_image_shm < 0)
    {
        printf("I_InitDDSS: Unable to create DDS topic: %s\n", dds_strretcode(-topic_image_shm));
        dds_destroy();
        return;
    }

    topic_obj_poses = dds_create_topic (participant, &geometry_msgs_msg_dds__PoseArray__desc, OBJ_POSES_DDS_PUB_TOPIC_NAME, NULL, NULL);
    if (topic_obj_poses < 0)
    {
        printf("I_InitDDSS: Unable to create DDS topic: %s\n", dds_strretcode(-topic_obj_poses));
        dds_destroy();
        return;
    }

    reader_input = dds_create_reader(participant, topic_input, NULL, NULL);
    if (reader_input < 0)
    {
        printf("I_InitDDS: Unable to create DDS reader: %s\n", dds_strretcode(-reader_input));
        dds_destroy();
        return;
    }

    writer_image = dds_create_writer(participant, topic_image, image_qos, NULL);
    if (writer_image < 0)
    {
        printf("I_InitDDS: Unable to create DDS writer: %s\n", dds_strretcode(-writer_image));
        dds_destroy();
        return;
    }

    writer_image_shm = dds_create_writer(participant, topic_image_shm, image_qos, NULL);
    if (writer_image_shm < 0)
    {
        printf("I_InitDDS: Unable to create DDS writer: %s\n", dds_strretcode(-writer_image_shm));
        dds_destroy();
        return;
    }

    writer_obj_poses = dds_create_writer(participant, topic_obj_poses, image_qos, NULL);
    if (writer_obj_poses < 0)
    {
        printf("I_InitDDS: Unable to create DDS writer: %s\n", dds_strretcode(-writer_obj_poses));
        dds_destroy();
        return;
    }

    /* Initialize input data buffer */
    for (int i = 0; i < READ_BUF_SIZE; i++)
    {
        read_buf[i] = sensor_msgs_msg_GamePad__alloc();
        /* Set buffer data to zero */
        memset(read_buf[i], 0, sizeof(sensor_msgs_msg_GamePad));
    }

    // Initialize object pose data
    pose_array.poses._buffer = dds_sequence_geometry_msgs_msg_dds__Pose__allocbuf(MAX_NUM_POSES);
    pose_array.poses._maximum = MAX_NUM_POSES;
    pose_array.poses._length = 0;
    pose_array.poses._release = false;
    pose_array.header.frame_id = "map";

    // Initialize image
    memset(&image, 0, sizeof(sensor_msgs_msg_dds__Image_));
    image.header.frame_id = "doom_image";
    image.height = DOOMGENERIC_RESY;
    image.width = DOOMGENERIC_RESX;
    image.data._release = false;
    image.data._buffer = (uint8_t*)DG_ScreenBuffer;
    image.data._maximum = DOOMGENERIC_RESY * DOOMGENERIC_RESX * 4;
    image.data._length = DOOMGENERIC_RESY * DOOMGENERIC_RESX * 4;
    image.is_bigendian = false;
    image.step = DOOMGENERIC_RESX * sizeof(uint32_t);
    image.encoding = "bgra8";

    // Initialize shared memory image
    memset(&image_shm, 0, sizeof(sensor_msgs_msg_dds__Image_shm_));
    image_shm.header.frame_id = "doom_image_shm";
    image_shm.height = DOOMGENERIC_RESY;
    image_shm.width = DOOMGENERIC_RESX;
    image_shm.is_bigendian = false;
    image_shm.step = DOOMGENERIC_RESX * sizeof(uint32_t);
    image_shm.encoding = "bgra8";
    image_shm.data.name = shm_name;
    image_shm.data.size = DOOMGENERIC_RESY * DOOMGENERIC_RESX * 4;

    // Create shared memory
    shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1)
        printf("Error creating shared memory!\n");
    else
        printf("Created shared memory, file descriptor: %i\n", shm_fd);

    if (shm_fd != -1)
    {
        ftruncate(shm_fd, image_shm.data.size);
        p_shm = mmap(0, image_shm.data.size, PROT_WRITE, MAP_SHARED, shm_fd, 0);
    }

    if (p_shm == MAP_FAILED)
        printf("Error mapping shared memory!\n");
    else
        printf("Shared memory mapped to address %lx\n", (long)p_shm);

    printf("I_InitDDS: DDS initialized\n");
}



double fixed2double(fixed_t input)
{
    return ((double)input / (double)(1 << FRACBITS));
}

void V_PublishDDSImage(void)
{
    if (p_shm != MAP_FAILED)
    {
        // Copy rendered image to shared memory 
        memcpy(p_shm, DG_ScreenBuffer, image_shm.data.size);

        // Publish shared memory image
        if (dds_write(writer_image_shm, &image_shm) < 0)
            printf("Error publishing image : %s\n", dds_strretcode(-writer_image_shm));
    }

    // Set screen buffer alpha channel
    for(int i = 3; i < DOOMGENERIC_RESX * DOOMGENERIC_RESY * 4; i = i + 4)
    {
        ((uint8_t*)DG_ScreenBuffer)[i] = 0xFF;
    }
    
    // Check some rgb stuff
    uint8_t *bgra = (uint8_t*)DG_ScreenBuffer;
    printf("b: %i g: %i r: %i a: %i\n", bgra[0], bgra[1], bgra[2], bgra[3]);

    // Publish image
    struct timespec res;
    clock_gettime(CLOCK_REALTIME,&res);
    image.header.stamp.nanosec = res.tv_nsec;
    image.header.stamp.sec = res.tv_sec;
    if (dds_write(writer_image, &image) < 0)
        printf("Error publishing image : %s\n", dds_strretcode(-writer_image));

    pose_array.poses._length = 0;
    for (int i = 0; i < numsectors; i++)
    {
        mobj_t *t = sectors[i].thinglist;
        while (t && (pose_array.poses._length < MAX_NUM_POSES))
        {
            switch (t->type)
            {
            case MT_SHOTGUY:
            case MT_TROOP:
            case MT_POSSESSED:
            case MT_PLAYER:
                // printf("Enemy pos X = %f Y = %f Height = %f\n", fixed2double(t->x), fixed2double(t->y), fixed2double(t->floorz));
                pose_array.poses._buffer[pose_array.poses._length].position.x = fixed2double(t->x) / 1000.0;
                pose_array.poses._buffer[pose_array.poses._length].position.y = fixed2double(t->y) / 1000.0;
                pose_array.poses._buffer[pose_array.poses._length].position.z = fixed2double(t->z) / 1000.0;
                RPY2Quat(&pose_array.poses._buffer[pose_array.poses._length].orientation, 0, 0, fixed2double(t->angle) / 32768 * M_PI);
                pose_array.poses._length++;
                break;
            
            default:
                break;
            }

            if (t->type == MT_PLAYER)
                ;//printf("Angle: %f\n", fixed2double(t->angle) / 32768);
                
            t = t->snext;
        }
    }

    // Publish pose array of objects
    if (dds_write(writer_obj_poses, &pose_array) < 0)
        printf("Error publishing poses : %s\n", dds_strretcode(-writer_obj_poses));



}

void I_UpdateDDSInput(void)
{
    event_t ev;
    dds_return_t input_read_status = -1;
    sensor_msgs_msg_GamePad *input;

    // Get input from DDS
    while ((input_read_status = dds_take(reader_input, read_buf, sample_infos, 1, 1)) > 0)
        ;    

    memset(&ev, 0 , sizeof(event_t));
    ev.type = ev_joystick;

    if (input_read_status >= 0)
    {
    input = read_buf[0];

    // Event-related data that depends on the type of event:
    //
    // ev_keydown/ev_keyup:
    //    data1: Key code (from doomkeys.h) of the key that was
    //           pressed or released.
    //    data2: Ascii text of the character that was pressed,
    //           shifted appropriately (eg. '$' if 4 was pressed
    //           while shift was held).
    //
    // ev_mouse:
    //    data1: Bitfield of buttons currently held down.
    //           (bit 0 = left; bit 1 = right; bit 2 = middle).
    //    data2: X axis mouse movement (turn).
    //    data3: Y axis mouse movement (forward/backward).
    //
    // ev_joystick:
    //    data1: Bitfield of buttons currently pressed.
    //    data2: X axis mouse movement (turn).
    //    data3: Y axis mouse movement (forward/backward).
    //    data4: Third axis mouse movement (strafe).

    //
    // Button/action code definitions.
    //
    // typedef enum
    // {
    //     // Press "Fire".
    //     BT_ATTACK		= 1,
    //     // Use button, to open doors, activate switches.
    //     BT_USE		= 2,
    // 
    //     // Flag: game events, not really buttons.
    //     BT_SPECIAL		= 128,
    //     BT_SPECIALMASK	= 3,
    //     
    //     // Flag, weapon change pending.
    //     // If true, the next 3 bits hold weapon num.
    //     BT_CHANGE		= 4,
    //     // The 3bit weapon mask and shift, convenience.
    //     BT_WEAPONMASK	= (8+16+32),
    //     BT_WEAPONSHIFT	= 3,
    // 
    //     // Pause the game.
    //     BTS_PAUSE		= 1,
    //     // Save the game at each console.
    //     BTS_SAVEGAME	= 2,
    // 
    //     // Savegame slot numbers
    //     //  occupy the second byte of buttons.    
    //     BTS_SAVEMASK	= (4+8+16),
    //     BTS_SAVESHIFT 	= 2,
    //   
    // } buttoncode_t;


    ev.data1 |= (int)(input->RT > 10.0) << 0;   // Right trigger = attack
    ev.data1 |= (int)(input->y) << 1;           // Y button = use

    if ((int)input->RS_X > 10)
        ev.data2 = (int)input->RS_X - 10;
    else if ((int)input->RS_X < -10)
        ev.data2 = (int)input->RS_X + 10;

    if ((int)input->LS_Y > 10)
        ev.data3 = (int)input->LS_Y - 10;
    else if ((int)input->LS_Y < -10)
        ev.data3 = (int)input->LS_Y + 10;

    if ((int)input->LS_X > 10)
        ev.data4 = (int)input->LS_X - 10;
    else if ((int)input->LS_X < -10)
        ev.data4 = (int)input->LS_X + 10;

    }

    D_PostEvent(&ev);
}

static void dds_destroy()
{
    if (participant < 0)
        return;

    dds_return_t rc = dds_delete (participant);
    if (rc != DDS_RETCODE_OK)
        ; /* TODO: Do something useful on error */

    /* Free read buffer */
    for (int i = 0; i < READ_BUF_SIZE; i++)
    {
        if (read_buf[i] != NULL)
           sensor_msgs_msg_GamePad_free(read_buf[i], DDS_FREE_ALL);
    }

    if (shm_fd != -1)
        shm_unlink(shm_name);

    for (int i = 0; i < pose_array.poses._maximum; i++)
    {
        if (pose_array.poses._buffer != NULL)
            geometry_msgs_msg_dds__PoseArray__free(pose_array.poses._buffer, DDS_FREE_ALL);
    }
}
