/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include "MulticopterRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;
using namespace time_literals;
using math::radians;

//========== omnirotor functions ===============//
Matrix<float, 3, 3> skew_cross(Vector3f v)
{
    Matrix<float, 3, 3> m;
    m(0,0) = 0; m(0,1) = -v(2); m(0,2) = v(1);
    m(1,0) = v(2); m(1,1) = 0; m(1,2) = -v(0);
    m(2,0) = -v(1); m(2,1) = v(0); m(2,2) = 0;

    return m;
}
//========== omnirotor functions END ===========//

MulticopterRateControl::MulticopterRateControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_actuators_0_pub(vtol ? ORB_ID(actuator_controls_virtual_mc) : ORB_ID(actuator_controls_0)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
}

MulticopterRateControl::~MulticopterRateControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle_angular_velocity callback registration failed!");
		return false;
	}

	return true;
}

void
MulticopterRateControl::parameters_updated()
{
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	_rate_control.setGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));


	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled_by_val(_param_cbrk_rate_ctrl.get(), CBRK_RATE_CTRL_KEY);

	//set filter parameters
	pos_fork_filter.setParameters(0.04, 0.1);
}

float
MulticopterRateControl::get_landing_gear_state()
{
	// Only switch the landing gear up if we are not landed and if
	// the user switched from gear down to gear up.
	// If the user had the switch in the gear up position and took off ignore it
	// until he toggles the switch to avoid retracting the gear immediately on takeoff.
	if (_landed) {
		_gear_state_initialized = false;
	}

	float landing_gear = landing_gear_s::GEAR_DOWN; // default to down

	if (_manual_control_setpoint.gear_switch == manual_control_setpoint_s::SWITCH_POS_ON && _gear_state_initialized) {
		landing_gear = landing_gear_s::GEAR_UP;

	} else if (_manual_control_setpoint.gear_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
		// Switching the gear off does put it into a safe defined state
		_gear_state_initialized = true;
	}

	return landing_gear;
}

void
MulticopterRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		// grab corresponding vehicle_angular_acceleration immediately after vehicle_angular_velocity copy
		vehicle_angular_acceleration_s v_angular_acceleration{};
		_vehicle_angular_acceleration_sub.copy(&v_angular_acceleration);

		const hrt_abstime now = angular_velocity.timestamp_sample;

		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		const Vector3f angular_accel{v_angular_acceleration.xyz};
		const Vector3f rates{angular_velocity.xyz};

		/* check for updates in other topics */
		_v_control_mode_sub.update(&_v_control_mode);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}

		_vehicle_status_sub.update(&_vehicle_status);

		const bool manual_control_updated = _manual_control_setpoint_sub.update(&_manual_control_setpoint);

		// generate the rate setpoint from sticks?
		bool manual_rate_sp = false;

		if (_v_control_mode.flag_control_manual_enabled &&
		    !_v_control_mode.flag_control_altitude_enabled &&
		    !_v_control_mode.flag_control_velocity_enabled &&
		    !_v_control_mode.flag_control_position_enabled) {

			// landing gear controlled from stick inputs if we are in Manual/Stabilized mode
			//  limit landing gear update rate to 10 Hz
			if ((now - _landing_gear.timestamp) > 100_ms) {
				_landing_gear.landing_gear = get_landing_gear_state();
				_landing_gear.timestamp = hrt_absolute_time();
				_landing_gear_pub.publish(_landing_gear);
			}

			if (!_v_control_mode.flag_control_attitude_enabled) {
				manual_rate_sp = true;
			}

			// Check if we are in rattitude mode and the pilot is within the center threshold on pitch and roll
			//  if true then use published rate setpoint, otherwise generate from manual_control_setpoint (like acro)
			if (_v_control_mode.flag_control_rattitude_enabled) {
				manual_rate_sp =
					(fabsf(_manual_control_setpoint.y) > _param_mc_ratt_th.get()) ||
					(fabsf(_manual_control_setpoint.x) > _param_mc_ratt_th.get());
			}

		} else {
			_landing_gear_sub.update(&_landing_gear);
		}

		if (manual_rate_sp) {
			if (manual_control_updated) {

				// manual rates control - ACRO mode
				const Vector3f man_rate_sp{
					math::superexpo(_manual_control_setpoint.y, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(-_manual_control_setpoint.x, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(_manual_control_setpoint.r, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

				_rates_sp = man_rate_sp.emult(_acro_rate_max);
				_thrust_sp = _manual_control_setpoint.z;

				// publish rate setpoint
				vehicle_rates_setpoint_s v_rates_sp{};
				v_rates_sp.roll = _rates_sp(0);
				v_rates_sp.pitch = _rates_sp(1);
				v_rates_sp.yaw = _rates_sp(2);
				v_rates_sp.thrust_body[0] = 0.0f;
				v_rates_sp.thrust_body[1] = 0.0f;
				v_rates_sp.thrust_body[2] = -_thrust_sp;
				v_rates_sp.timestamp = hrt_absolute_time();

				_v_rates_sp_pub.publish(v_rates_sp);
			}

		} else {
			// use rates setpoint topic
			vehicle_rates_setpoint_s v_rates_sp;

			if (_v_rates_sp_sub.update(&v_rates_sp)) {
				_rates_sp(0) = v_rates_sp.roll;
				_rates_sp(1) = v_rates_sp.pitch;
				_rates_sp(2) = v_rates_sp.yaw;
				_thrust_sp = -v_rates_sp.thrust_body[2];
			}
		}
		//==================================================================//
		//============== here starts your main code/loop ===================//
		//=== check your subscription here
		//get update on dynamixels' position
		// update_dynxl_pos();

		//read the rc channels you need
		_rc_channels_sub.update(&_rc_channels);
		//put channel 8 value into debug vector
		//_dynxls_d.x = _rc_channels.channels[5];

		// run the rate controller
		if (_v_control_mode.flag_control_rates_enabled && !_actuators_0_circuit_breaker_enabled) {

			// reset integral if disarmed
			if (!_v_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				_rate_control.resetIntegral();
			}

			// update saturation status from mixer feedback
			if (_motor_limits_sub.updated()) {
				multirotor_motor_limits_s motor_limits;

				if (_motor_limits_sub.copy(&motor_limits)) {
					MultirotorMixer::saturation_status saturation_status;
					saturation_status.value = motor_limits.saturation_status;

					_rate_control.setSaturationStatus(saturation_status);
				}
			}

			// run rate controller
			// const Vector3f att_control = _rate_control.update(rates, _rates_sp, angular_accel, dt, _maybe_landed || _landed);
			Vector3f att_control = _rate_control.update(rates, _rates_sp, angular_accel, dt, _maybe_landed || _landed);

			if ((_rc_channels.channels[7] < (float)-0.5))
			{
				if (_rc_channels.channels[8] < (float)-0.5)
				{
					// att_control(2) = 0; //ignore yaw
					// inverted_ctrl(att_control);
					mode = 1; //inverted
				}
				else if (_rc_channels.channels[8] > (float)0.3)
				{
					// att_control(1) = 0; //ignore pitch
					// att_control(2) = -att_control(2); //invert yaw
					// hanging_ctrl(att_control);
					mode = 5; //restore_and_hanging
				}
				else if ((_rc_channels.channels[8] > (float)-0.2) && (_rc_channels.channels[8] < (float)0.2))
				{
					// att_control(1) = 0; //ignore pitch
					// att_control(2) = -att_control(2); //invert yaw
					// hanging_ctrl(att_control);
					mode = 2; //hanging
				}
			}
			else
			{
				//ignore all rate control
				att_control(0) = 0;
				att_control(1) = 0;
				att_control(2) = 0;

				if ((_rc_channels.channels[7] > (float)-0.2) && (_rc_channels.channels[7] < (float)0.2))
				{
					// ground_restore(att_control);
					mode = 3; //ground restore
				}
				else
				{
					// free_rotation_ctrl(att_control);
					// ground_ctrl(att_control);
					// ground_ctrl_position(att_control);
					mode = 4; //ground control or free rotation
				}



			}

			//Account for mode change
			if (mode != mode_old)
			{
				continuity_guarantee();
			}

			switch (mode)
			{
			case 1:
				inverted_ctrl(att_control);
				break;
			case 2:
				att_control(2) = -att_control(2); //invert yaw
				hanging_ctrl(att_control);
				break;
			case 3:
				//ignore all rate control
				att_control(0) = 0;
				att_control(1) = 0;
				att_control(2) = 0;
				ground_restore(att_control);
				break;
			case 4:
				//ignore all rate control
				att_control(0) = 0;
				att_control(1) = 0;
				att_control(2) = 0;
				// free_rotation_ctrl(att_control);
				ground_ctrl(att_control);
				break;
			case 5:
				att_control(2) = -att_control(2); //invert yaw
				restore_and_hanging(att_control);
				break;

			default:
				inverted_ctrl(att_control);
				break;
			}

			//store the mode that runned this loop
			mode_old = mode;

		//==================================================================//
		//============== here ENDS your main code/loop ===================//


			// publish rate controller status
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status);

			// publish actuator controls
			actuator_controls_s actuators{};
			actuators.control[actuator_controls_s::INDEX_ROLL] = PX4_ISFINITE(att_control(0)) ? att_control(0) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_PITCH] = PX4_ISFINITE(att_control(1)) ? att_control(1) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_YAW] = PX4_ISFINITE(att_control(2)) ? att_control(2) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_THROTTLE] = PX4_ISFINITE(_thrust_sp) ? _thrust_sp : 0.0f;
			actuators.control[actuator_controls_s::INDEX_LANDING_GEAR] = (float)_landing_gear.landing_gear;
			actuators.timestamp_sample = angular_velocity.timestamp_sample;

			// scale effort by battery status if enabled
			if (_param_mc_bat_scale_en.get()) {
				if (_battery_status_sub.updated()) {
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status)) {
						_battery_status_scale = battery_status.scale;
					}
				}

				if (_battery_status_scale > 0.0f) {
					for (int i = 0; i < 4; i++) {
						actuators.control[i] *= _battery_status_scale;
					}
				}
			}

			actuators.timestamp = hrt_absolute_time();
			_actuators_0_pub.publish(actuators);

		} else if (_v_control_mode.flag_control_termination_enabled) {
			if (!_vehicle_status.is_vtol) {
				// publish actuator controls
				actuator_controls_s actuators{};
				actuators.timestamp = hrt_absolute_time();
				_actuators_0_pub.publish(actuators);
			}
		}
	}

	perf_end(_loop_perf);
}

int MulticopterRateControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterRateControl *instance = new MulticopterRateControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MulticopterRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

void MulticopterRateControl::update_dynxl_pos()
{
	if (_debug_vect_sub.update(&_dynxls))
	{
			if (_dynxls.z >= (float)1.95 && _dynxls.z <= (float)2.05)
			{
				// dyxl_pos1 = _dynxls.x;
				// dyxl_pos2 = _dynxls.y; //If not commentted, it stops working.
			}
	}
}

Vector3f MulticopterRateControl::torque_CG_map_inv(Vector3f Ce_ang_)
{
	//== Do here the mapping to a desired thrust
	Vector3f r_cg2; //The vector from the cg of the system to the cg of the core
	r_cg2(0) = 0; r_cg2(1) = 0; r_cg2(2) = 0.051;
	//define the map
	Matrix<float, 3, 3> map_cross;
	map_cross = skew_cross(r_cg2);
	//invert the map
	Matrix<float, 3, 3> map_cross_inv;
	map_cross_inv = geninv(map_cross);
	//apply the inverse to the total desired torque
	Vector3f T_dtal;
	T_dtal = map_cross_inv*Ce_ang_;

	return T_dtal;
}

Vector3f MulticopterRateControl::rotor_IK_no_sing(Vector3f T_d_)
{
	//normalize T_d_
	Vector3f T_d_norm =  T_d_.normalized();

	//== calculate the fork ==//
	//Get the rotation matrix of the fork link w.r.t to the base link
	// define an euler angle (Body 3(yaw)-2(pitch)-1(roll) rotation)
	float yaw_fork = (float)dyxl_pos1;  //rotation on z axis
	yaw_fork = 0; //This was messing things when using ground mode
	Eulerf euler_fork(0.0, 0.0, yaw_fork); //euler angles
	Dcmf R_fork(euler_fork); //creates the rotation matrix from euler angles
	//express T_d_norm on the fork frame
	Vector3f T_d_on_fork_xy;
	T_d_on_fork_xy = R_fork.T()*T_d_norm;
	T_d_on_fork_xy(2) = 0; //Project into xy plane
	Vector3f T_d_on_fork_xy_norm = T_d_on_fork_xy.normalized();
	//calculate the cross product of fork_y with the desired direction
	Vector3f fork_yxTdp = skew_cross(Vector3f(0,-1,0))*T_d_on_fork_xy_norm;
	//calculate theta fork
	//calculate the desired angle to move the fork based on the two axis of symmetry and two eq points joint model
	// Matrix<float, 1, 1> theta_fork_ = (fork_yxTdp.T()*Vector3f(0,0,-1))*(T_d_on_fork_xy_norm.T()*Vector3f(0,1,0));
	Matrix<float, 1, 1> atany_ = (fork_yxTdp.T()*Vector3f(0,0,-1));
	Matrix<float, 1, 1> atanx_ = (T_d_on_fork_xy_norm.T()*Vector3f(0,-1,0));
	float atany = -atany_(0,0);
	float atanx = atanx_(0,0);
	float theta_fork = atan(atany/atanx);
	//pos_fork_filter.update(theta_fork);
	//theta_fork = pos_fork_filter.getState();
	// float theta_fork = (theta_fork_(0,0));
	//== calculate the fork END ==//

	//== calculate the core ==//
	//get the direction of interest of the child body w.r.t the parent frame (Z vector from the core frame, w.r.t the fork frame)
	// define an euler angle (Body 3(yaw)-2(pitch)-1(roll) rotation)
	float pitch_core = (float)dyxl_pos2;  //rotation on y axis
	Eulerf euler_core(0.0, pitch_core, 0.0); //euler angles
	Dcmf fork_R_core(euler_core); //creates the rotation matrix from euler angles
	//update fork matrix here, based on previous computations
	Eulerf euler_forkc(0.0, 0.0, theta_fork); //euler angles
	Dcmf R_forkc(euler_forkc); //creates the rotation matrix from euler angles
	//Express the desired direction w.r.t the parent frame (child frame??)
  	Vector3f T_d_on_core = fork_R_core.T()*R_fork.T()*T_d_norm;
	//Project the desired force onto the working plane of the joint (yz plane of the fork)
	Vector3f T_d_on_core_yz = T_d_on_core;
	T_d_on_core_yz(0) = 0;
	//Normalize it
	Vector3f T_d_on_core_yz_norm = T_d_on_core_yz.normalized();
	//calculate the cross product of the direction of interest of the child body with the desired direction
	Vector3f core_ZxTdp = skew_cross(Vector3f(0,0,-1))*T_d_on_core_yz_norm;
	//calculate the desired angle to move the core based on the one axis of symmetry and one eq points joint model
	Matrix<float, 1, 1> atan2y_ = core_ZxTdp.T()*Vector3f(1,0,0);
	float atan2y = atan2y_(0,0);
	Matrix<float, 1, 1> atan2x_ = T_d_on_core_yz_norm.T()*Vector3f(0,0,-1);
	float atan2x = atan2x_(0,0);
	float theta_core = atan2(atan2y, atan2x);

	Vector3f u_out;
	u_out(0) = theta_fork + (float)dyxl_pos1;
	// dyxl_pos1 = theta_fork;
	u_out(1) = -theta_core;// + (float)dyxl_pos2;
	// dyxl_pos2 = theta_core;
	u_out(2) = 1; //my convention to id this msg on mavlink comm

	return u_out;
}

void MulticopterRateControl::inverted_ctrl(Vector3f att_control_)
{
	//Do here the mapping to a desired thrust
	Vector3f T_dtal;
	T_dtal = torque_CG_map_inv(att_control_);
	//Add the thrust on z direction
	T_dtal(2) = T_dtal(2) - _thrust_sp - 17;

	//Do the IK for the rotor
	Vector3f u;
	u = rotor_IK_no_sing(T_dtal);

	_dynxls_d.x = 0; //for not using the fork dof
	_dynxls_d.x = grd_mode_pos1_old; //for smooth integration with ground mode
	_dynxls_d.y = u(1) + (float)grd_mode_pos2_next_vertical;
	_dynxls_d.z = u(2);
	_dynxls_d.timestamp = hrt_absolute_time();
	_debug_vect_pub.publish(_dynxls_d);

}

void MulticopterRateControl::hanging_ctrl(Vector3f att_control_)
{
	//Do here the mapping to a desired thrust
	Vector3f T_dtal;
	T_dtal = torque_CG_map_inv(att_control_);
	//Add the thrust on z direction
	T_dtal(2) = -T_dtal(2) - _thrust_sp - 17;

	//Do the IK for the rotor
	Vector3f u;
	u = rotor_IK_no_sing(T_dtal);

	//publish only if armed
	_dynxls_d.x = 0; //for not using the fork dof
	_dynxls_d.x = grd_mode_pos1_old; //for smooth integration with ground mode
	_dynxls_d.y = -u(1) + (float)MATH_PI + (float)grd_mode_pos2_next_vertical;
	_dynxls_d.z = u(2);
	_dynxls_d.timestamp = hrt_absolute_time();
	_debug_vect_pub.publish(_dynxls_d);

}

void MulticopterRateControl::ground_restore(Vector3f att_control_)
{
	_vehicle_attitude_sub.update(&vehicle_att);

	Quatf q(vehicle_att.q);
	Dcmf R(q);
	Vector3f G_z;
	G_z = R.T()*Vector3f(0,0,1); //z vector for ground frame. Direction of gravity, as seen from the vehicle frame.

	float theta_fork = atan(G_z(0)/G_z(1)); //get the angle between projected z_ground and y axis of vehicle frame
	float theta_core = atan2(-G_z(1),-G_z(2)) + (float)MATH_PI; //get the angle between the negative of z ground and y of vehicle
	if (fabsf(G_z(2)) > 0.94f)
	{
		theta_fork = 0;
		// theta_core = 0;
	}
	grd_mode_pos1_old = grd_mode_pos1_old + (double)theta_fork*0.005; //Gravity following with a P controller


	_dynxls_d.x = 0; //for not using the fork dof
	_dynxls_d.x = grd_mode_pos1_old;
	_dynxls_d.y = theta_core + (float)grd_mode_pos2_next_vertical;

	pos2_old = (float)_dynxls_d.y;

	_dynxls_d.z = 1;
	_dynxls_d.timestamp = hrt_absolute_time();
	_debug_vect_pub.publish(_dynxls_d);
}

void MulticopterRateControl::restore_and_hanging(Vector3f att_control_)
{
	//Unifying ground restore and hanging flying mode
	_vehicle_attitude_sub.update(&vehicle_att);

	Quatf q(vehicle_att.q);
	Dcmf R(q);
	Vector3f G_z;
	G_z = R.T()*Vector3f(0,0,1); //z vector for ground frame. Direction of gravity, as seen from the vehicle frame.

	//Just switch to hanging control after a certaing tolerance in alighment with gravity (cos(25 deg) ~ 0.9)
	if (fabsf(G_z(2)) > 0.77f)
	{
		hanging_ctrl(att_control_);
	}
	else
	{
		ground_restore(att_control_);
	}

}

void MulticopterRateControl::ground_ctrl(Vector3f att_control_)
{
	_vehicle_attitude_sub.update(&vehicle_att);
	Quatf q(vehicle_att.q);
	Dcmf R(q);
	Vector3f G_z;
	G_z = R.T()*Vector3f(0,0,1); //z vector for ground frame. Direction of gravity, as seen from the vehicle frame.
	if (G_z(2) < 0)
	{
		grd_mode_pos1_old = (double)_rc_channels.channels[1]*0.01 + grd_mode_pos1_old;
		_dynxls_d.y = _rc_channels.channels[10]*(float)MATH_PI/(float)2.0 + (float)MATH_PI/(float)2.0 + _rc_channels.channels[2]*(float)MATH_PI + (float)grd_mode_pos2_next_vertical;
		_test_motors.value = _rc_channels.channels[10];
	}
	else
	{
		grd_mode_pos1_old = (double)_rc_channels.channels[1]*-0.01 + grd_mode_pos1_old; //keep radio input the same for both hanging and inverted pose
		_dynxls_d.y = _rc_channels.channels[10]*(float)MATH_PI/(float)2.0 + (float)MATH_PI/(float)2.0 - _rc_channels.channels[2]*(float)MATH_PI + (float)grd_mode_pos2_next_vertical;
	}
	_dynxls_d.x = grd_mode_pos1_old;
	// _dynxls_d.y = _rc_channels.channels[10]*(float)MATH_PI/(float)2.0 + (float)MATH_PI/(float)2.0 - _rc_channels.channels[2]*(float)MATH_PI + (float)grd_mode_pos2_next_vertical;
	_dynxls_d.z = 1;
	_dynxls_d.timestamp = hrt_absolute_time();
	_debug_vect_pub.publish(_dynxls_d); //my convention to id this msg on mavlink comm

	//fill in _test_motor
	_test_motors.action = _test_motors.ACTION_RUN;
	_test_motors.motor_number = 3;
	_test_motors.timeout_ms = 0;
	_test_motors.timestamp = hrt_absolute_time();
	_test_motors.driver_instance = 0;
	_test_motor_pub.publish(_test_motors);

}

void MulticopterRateControl::ground_ctrl_position(Vector3f att_control_)
{
	// float MATH_PI = 3.1415;
	_vehicle_local_position_setpoint_sub.update(&_local_pos_setpoint);
	// grd_mode_pos1_old = 0;
	_dynxls_d.x = atan2(_local_pos_setpoint.thrust[0], _local_pos_setpoint.thrust[1]);
	_dynxls_d.y = (float)MATH_PI;
	_dynxls_d.z = 1;
	_dynxls_d.timestamp = hrt_absolute_time();
	_debug_vect_pub.publish(_dynxls_d); //my convention to id this msg on mavlink comm
}

void MulticopterRateControl::free_rotation_ctrl(Vector3f att_control_)
{
	// float MATH_PI = 3.1415;
	grd_mode_pos1_old = (double)_rc_channels.channels[1]*-0.01 + grd_mode_pos1_old;
	pos2_old = (double)_rc_channels.channels[2]*-0.01 + pos2_old;
	_dynxls_d.x = grd_mode_pos1_old;
	_dynxls_d.y = pos2_old;
	_dynxls_d.z = 1;
	_dynxls_d.timestamp = hrt_absolute_time();
	_debug_vect_pub.publish(_dynxls_d); //my convention to id this msg on mavlink comm

}

void MulticopterRateControl::continuity_guarantee()
{
	//Get next vertical position for core motor
	float rotation;
	rotation = (float)pos2_old/(float)MATH_PI;
	int i_f;
	if (rotation >= 0)
        {
		i_f = (int) (rotation + (float)0.5);
	}
    	else
        {
		i_f = (int) (rotation - (float)0.5);
	}

	if ((i_f%2)==0)
	{
		grd_mode_pos2_next_vertical = i_f*(float)MATH_PI;
	}
	else
	{
		grd_mode_pos2_next_vertical = (i_f+1)*(float)MATH_PI;
	}
}

int MulticopterRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_rate_control_omni", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_rate_control_omni_main(int argc, char *argv[])
{
	return MulticopterRateControl::main(argc, argv);
}
