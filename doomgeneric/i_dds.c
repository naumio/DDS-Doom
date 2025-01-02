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

#include "doomtype.h"
#include "d_event.h"
#include "i_system.h"

#include "m_config.h"
#include "m_misc.h"

#include "dds/dds.h"
#include "ROS_GamePad.h"

#define READ_BUF_SIZE 1
#define GAMEPAD_DDS_SUB_TOPIC_NAME "rt/dds_gamepad"

static dds_entity_t participant = -1;
static dds_entity_t topic_input = -1;
static dds_entity_t reader_input = -1;

static void *read_buf[READ_BUF_SIZE] = {0};
static dds_sample_info_t sample_infos[READ_BUF_SIZE] = {0};

static int weapon = 0;

static void dds_destroy();

void I_ShutdownDDS(void)
{
    dds_destroy();
}


void I_InitDDS(void)
{
    /* Register shudown callback */
    I_AtExit(I_ShutdownDDS, true);

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

    reader_input = dds_create_reader(participant, topic_input, NULL, NULL);
    if (reader_input < 0)
    {
        printf("I_InitDDS: Unable to create DDS topic: %s\n", dds_strretcode(-reader_input));
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


    printf("I_InitDDS: DDS initialized\n");
}


void I_UpdateDDS(void)
{
    event_t ev;
    dds_return_t rc = -1;
    sensor_msgs_msg_GamePad *input;

    // Get input from DDS
    while ((rc = dds_take(reader_input, read_buf, sample_infos, 1, 1)) > 0)
        ;    

    if (rc < 0)
        return;

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

    memset(&ev, 0 , sizeof(event_t));
    ev.type = ev_joystick;

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
}
