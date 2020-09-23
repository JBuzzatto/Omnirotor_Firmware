/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_mavlink_debug.cpp
 * Debug application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <poll.h>

#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/debug_array.h>

#include <lib/matrix/matrix/stdlib_imports.hpp>

extern "C" __EXPORT int omni_dynxl_ctrl_test_main(int argc, char *argv[]);

int omni_dynxl_ctrl_test_main(int argc, char *argv[])
{
	printf("Hello Debug!\n");

	// /* advertise named debug value */
	// struct debug_key_value_s dynxl_1;
	// strncpy(dynxl_1.key, "DyxPos1_d", 10);
	// dynxl_1.value = 0.0f;
	// orb_advert_t pub_dynxl_1 = orb_advertise(ORB_ID(debug_key_value), &dynxl_1);

	// struct debug_key_value_s dynxl_2;
	// strncpy(dynxl_2.key, "DyxPos2_d", 10);
	// dynxl_2.value = 0.0f;
	// orb_advert_t pub_dynxl_2 = orb_advertise(ORB_ID(debug_key_value), &dynxl_2);

	// /* advertise indexed debug value */
	// struct debug_value_s dbg_ind;
	// dbg_ind.ind = 42;
	// dbg_ind.value = 0.5f;
	// orb_advert_t pub_dbg_ind = orb_advertise(ORB_ID(debug_value), &dbg_ind);

	/* advertise debug vect */
	struct debug_vect_s dynxls_d;
	strncpy(dynxls_d.name, "DyxPos_d", 10);
	dynxls_d.x = 1.0f;
	dynxls_d.y = 2.0f;
	dynxls_d.z = 3.0f;
	orb_advert_t pub_dynxls_d = orb_advertise(ORB_ID(debug_vect), &dynxls_d);

	// /* advertise debug array */
	// struct debug_array_s dbg_array;
	// dbg_array.id = 1;
	// strncpy(dbg_array.name, "dbg_array", 10);
	// orb_advert_t pub_dbg_array = orb_advertise(ORB_ID(debug_array), &dbg_array);

	//subscribe to the topic too
	int debug_sub_fd = orb_subscribe(ORB_ID(debug_vect));
	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
   		 { .fd = debug_sub_fd,   .events = POLLIN },
	};

	int error_counter = 0;
	while (true)
	{
		/* wait for debug_key_value for 1000 ms (1 second) */
    		int poll_ret = px4_poll(fds, 1, 2);
		//warnx("on the loop to receive values...");
		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a 2 mili seconds");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN)
			{
				/* obtained data for the first file descriptor */
				struct debug_vect_s dynxls;

				/* copy data into local buffer */
				orb_copy(ORB_ID(debug_vect), debug_sub_fd, &dynxls);

				/* filter message based on its key attribute */
				// if (strcmp(dynxls.name, "DyxPos") == 0)
				// {
				// 	//PX4_INFO("DyxPos 1:\t%8.4f", (double)dynxls.x);
				// 	//PX4_INFO("DyxPos 2:\t%8.4f", (double)dynxls.y);
				// }
				//Filter msg based on its z value (z=2 for message from arduino with current dynamixels position)
				if (dynxls.z >= (float)1.95 && dynxls.z <= (float)2.05)
				{
					PX4_INFO("DyxPos 1:\t%8.4f", (double)dynxls.x);
					PX4_INFO("DyxPos 2:\t%8.4f", (double)dynxls.y);
				}


			}
		}
		int value_counter = 0;

		uint64_t timestamp_us = hrt_absolute_time();
		float val = (M_PI)*(matrix::sin((timestamp_us/1000000.0)*0.5));

		// /* send one named value */
		// dynxl_1.value = val;
		// dynxl_1.timestamp = timestamp_us;
		// orb_publish(ORB_ID(debug_key_value), pub_dynxl_1, &dynxl_1);

		// dynxl_2.value = val;
		// dynxl_2.timestamp = timestamp_us;
		// orb_publish(ORB_ID(debug_key_value), pub_dynxl_2, &dynxl_2);

		// /* send one indexed value */
		// dbg_ind.value = 0.5f * value_counter;
		// dbg_ind.timestamp = timestamp_us;
		// orb_publish(ORB_ID(debug_value), pub_dbg_ind, &dbg_ind);

		/* send one vector */
		dynxls_d.x = val;
		dynxls_d.y = val;
		dynxls_d.z = 1;
		dynxls_d.timestamp = timestamp_us;
		orb_publish(ORB_ID(debug_vect), pub_dynxls_d, &dynxls_d);

		//mavlink stream -s DEBUG_VECT -r 1 -d /dev/ttyS1

		// /* send one array */
		// for (size_t i = 0; i < debug_array_s::ARRAY_SIZE; i++)
		// {
		// 	dbg_array.data[i] = value_counter + i * 0.01f;
		// }

		// dbg_array.timestamp = timestamp_us;
		// orb_publish(ORB_ID(debug_array), pub_dbg_array, &dbg_array);

		//warnx("sent one more value..");

		value_counter++;
		px4_usleep(2500);

	}
	PX4_INFO("exiting");
	return 0;
}
