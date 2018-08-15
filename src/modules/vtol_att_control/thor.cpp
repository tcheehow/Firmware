/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
* @file thor.cpp
*
* @author Low Jun En	<lowjunen@gmail.com>
*/

#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include "thor.h"
#include "vtol_att_control_main.h"

#define ARSP_YAW_CTRL_DISABLE 4.0f	// airspeed at which we stop controlling yaw during a front transition
#define THROTTLE_TRANSITION_MAX 0.25f	// maximum added thrust above last value in transition
#define PITCH_TRANSITION_FRONT_P1 -1.1f	// pitch angle to switch to TRANSITION_P2
#define PITCH_TRANSITION_BACK -0.25f	// pitch angle to switch to MC

Thor::Thor(VtolAttitudeControl *attc) :
	VtolType(attc),
	_thrust_transition_start(0.0f),
	_yaw_transition(0.0f),
	_pitch_transition_start(0.0f)
{
	_vtol_schedule.flight_mode = MC_MODE;
	_vtol_schedule.transition_start = 0;

	_mc_roll_weight = 1.0f;
	_mc_pitch_weight = 1.0f;
	_mc_yaw_weight = 1.0f;

	_flag_was_in_trans_mode = false;

        _params_handles_thor.front_trans_dur_p2 = param_find("VT_TRANS_P2_DUR");

        _params_handles_thor.thor_rps_targ   = param_find("VT_THOR_RPS_TARG");
        _params_handles_thor.thor_rps_p      = param_find("VT_THOR_RPS_P");
        _params_handles_thor.thor_rps_i      = param_find("VT_THOR_RPS_I");
        _params_handles_thor.thor_coll_p     = param_find("VT_THOR_COLL_P");
        _params_handles_thor.thor_cyc_p      = param_find("VT_THOR_CYC_P");
        _params_handles_thor.thor_thr_trig   = param_find("VT_THOR_THR_TRIG");
        _params_handles_thor.thor_dir_trig   = param_find("VT_THOR_DIR_TRIG");
}

Thor::~Thor()
{

}

void
Thor::parameters_update()
{
	float v;

        /* vtol front transition phase 2 duration */
        param_get(_params_handles_thor.front_trans_dur_p2, &v);
        _params_thor.front_trans_dur_p2 = v;

        param_get(_params_handles_thor.thor_rps_targ, &v);
        _params_thor.thor_rps_targ = v;

        param_get(_params_handles_thor.thor_rps_p, &v);
        _params_thor.thor_rps_p = v;

        param_get(_params_handles_thor.thor_rps_i, &v);
        _params_thor.thor_rps_i = v;

        param_get(_params_handles_thor.thor_coll_p, &v);
        _params_thor.thor_coll_p = v;

        param_get(_params_handles_thor.thor_cyc_p, &v);
        _params_thor.thor_cyc_p = v;

        param_get(_params_handles_thor.thor_thr_trig, &v);
        _params_thor.thor_thr_trig = v;

        param_get(_params_handles_thor.thor_dir_trig, &v);
        _params_thor.thor_dir_trig = v;
}

void Thor::update_vtol_state()
{

	/* simple logic using a two way switch to perform transitions.
	 * after flipping the switch the vehicle will start tilting in MC control mode, picking up
	 * forward speed. After the vehicle has picked up enough and sufficient pitch angle the uav will go into FW mode.
	 * For the backtransition the pitch is controlled in MC mode again and switches to full MC control reaching the sufficient pitch angle.
	*/

	matrix::Eulerf euler = matrix::Quatf(_v_att->q);
	float pitch = euler.theta();

	if (!_attc->is_fixed_wing_requested()) {

		switch (_vtol_schedule.flight_mode) { // user switchig to MC mode
		case MC_MODE:
			break;

		case FW_MODE:
			_vtol_schedule.flight_mode 	= TRANSITION_BACK;
			_vtol_schedule.transition_start = hrt_absolute_time();
			break;

		case TRANSITION_FRONT_P1:
			// failsafe into multicopter mode
			_vtol_schedule.flight_mode = MC_MODE;
			break;

		case TRANSITION_BACK:

			// check if we have reached pitch angle to switch to MC mode
			if (pitch >= PITCH_TRANSITION_BACK) {
				_vtol_schedule.flight_mode = MC_MODE;
			}

			break;
		}

	} else {  // user switchig to FW mode

		switch (_vtol_schedule.flight_mode) {
		case MC_MODE:
			// initialise a front transition
			_vtol_schedule.flight_mode 	= TRANSITION_FRONT_P1;
			_vtol_schedule.transition_start = hrt_absolute_time();
			break;

		case FW_MODE:
			break;


                case TRANSITION_FRONT_P1: {
                    bool airspeed_condition_satisfied = _airspeed->indicated_airspeed_m_s >= _params->transition_airspeed;
                    airspeed_condition_satisfied |= _params->airspeed_disabled;

                    // check if we have reached airspeed  and pitch angle to switch to TRANSITION P2 mode
                    if ((airspeed_condition_satisfied && pitch <= PITCH_TRANSITION_FRONT_P1) || can_transition_on_ground()) {
                        _vtol_schedule.flight_mode = FW_MODE;
                    }
                    break;
                }

		case TRANSITION_BACK:
			// failsafe into fixed wing mode
			_vtol_schedule.flight_mode = FW_MODE;
			break;
		}
	}

        // map thor specific control phases to simple control modes
	switch (_vtol_schedule.flight_mode) {
	case MC_MODE:
		_vtol_mode = ROTARY_WING;
		_vtol_vehicle_status->vtol_in_trans_mode = false;
		_flag_was_in_trans_mode = false;
		break;

	case FW_MODE:
		_vtol_mode = FIXED_WING;
		_vtol_vehicle_status->vtol_in_trans_mode = false;
		_flag_was_in_trans_mode = false;
		break;

	case TRANSITION_FRONT_P1:
		_vtol_mode = TRANSITION_TO_FW;
		_vtol_vehicle_status->vtol_in_trans_mode = true;
		break;

	case TRANSITION_BACK:
		_vtol_mode = TRANSITION_TO_MC;
		_vtol_vehicle_status->vtol_in_trans_mode = true;
		break;
	}
}

void Thor::update_transition_state()
{
	float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;

	if (!_flag_was_in_trans_mode) {
		// save desired heading for transition and last thrust value
		_yaw_transition = _v_att_sp->yaw_body;
		//transition should start from current attitude instead of current setpoint
		matrix::Eulerf euler = matrix::Quatf(_v_att->q);
		_pitch_transition_start = euler.theta();
		_thrust_transition_start = _v_att_sp->thrust;
		_flag_was_in_trans_mode = true;
	}

	if (_vtol_schedule.flight_mode == TRANSITION_FRONT_P1) {

		// create time dependant pitch angle set point + 0.2 rad overlap over the switch value
		_v_att_sp->pitch_body = _pitch_transition_start	- fabsf(PITCH_TRANSITION_FRONT_P1 - _pitch_transition_start) *
					time_since_trans_start / _params->front_trans_duration;
		_v_att_sp->pitch_body = math::constrain(_v_att_sp->pitch_body, PITCH_TRANSITION_FRONT_P1 - 0.2f,
							_pitch_transition_start);

		_v_att_sp->thrust = _mc_virtual_att_sp->thrust;

		// disable mc yaw control once the plane has picked up speed
		if (_airspeed->indicated_airspeed_m_s > ARSP_YAW_CTRL_DISABLE) {
			_mc_yaw_weight = 0.0f;

		} else {
			_mc_yaw_weight = 1.0f;
		}

		_mc_roll_weight = 1.0f;
		_mc_pitch_weight = 1.0f;

	} else if (_vtol_schedule.flight_mode == TRANSITION_BACK) {

		if (!flag_idle_mc) {
			set_idle_mc();
			flag_idle_mc = true;
		}

		// create time dependant pitch angle set point stating at -pi/2 + 0.2 rad overlap over the switch value
		_v_att_sp->pitch_body = M_PI_2_F + _pitch_transition_start + fabsf(PITCH_TRANSITION_BACK + 1.57f) *
					time_since_trans_start / _params->back_trans_duration;
		_v_att_sp->pitch_body = math::constrain(_v_att_sp->pitch_body, -2.0f, PITCH_TRANSITION_BACK + 0.2f);


		_v_att_sp->thrust = _mc_virtual_att_sp->thrust;

		// keep yaw disabled
		_mc_yaw_weight = 0.0f;

		// smoothly move control weight to MC
		_mc_roll_weight = _mc_pitch_weight = time_since_trans_start / _params->back_trans_duration;

	}

	_mc_roll_weight = math::constrain(_mc_roll_weight, 0.0f, 1.0f);
	_mc_yaw_weight = math::constrain(_mc_yaw_weight, 0.0f, 1.0f);
	_mc_pitch_weight = math::constrain(_mc_pitch_weight, 0.0f, 1.0f);

	// compute desired attitude and thrust setpoint for the transition

	_v_att_sp->timestamp = hrt_absolute_time();
	_v_att_sp->roll_body = 0.0f;
	_v_att_sp->yaw_body = _yaw_transition;

	matrix::Quaternion<float> q_sp;
	q_sp = matrix::Eulerf(_v_att_sp->roll_body, _v_att_sp->pitch_body, _v_att_sp->yaw_body);
	memcpy(&_v_att_sp->q_d[0], &q_sp(0), sizeof(_v_att_sp->q_d));
}

void Thor::waiting_on_tecs()
{
	// copy the last trust value from the front transition
	_v_att_sp->thrust = _thrust_transition;
}

void Thor::update_mc_state()
{
	VtolType::update_mc_state();

	// set idle speed for rotary wing mode
	if (!flag_idle_mc) {
		set_idle_mc();
		flag_idle_mc = true;
	}
}

void Thor::update_fw_state()
{
	VtolType::update_fw_state();

	if (flag_idle_mc) {
		set_idle_fw();
		flag_idle_mc = false;
	}
}

/**
* Write data to actuator output topic.
*/
void Thor::fill_actuator_outputs()
{
    //    PX4_INFO("Monoco Enabled: %s", _v_control_mode->flag_control_monoco_enabled ? "true" : "false");

	switch (_vtol_mode) {
        case ROTARY_WING:
            if (_v_control_mode->flag_control_monoco_enabled) {

                // Time variables
                _monocopter_vars.new_time = hrt_absolute_time()/1000000.0f;
                float delta_t = (_monocopter_vars.new_time - _monocopter_vars.old_time);

                // Throttle variables
                float collective_cmd;
                float throttle_cmd;

                // Direction variables
                float des_dir;
                float des_mag;
                float cyclic_cmd;
                float heading;
                float time_bef;
                float time_aft;

                // Persistence of Vision
                float led_line;

                matrix::Eulerf euler  = matrix::Quatf(_v_att->q);
                _monocopter_vars.new_psi  = -euler.psi();
                _monocopter_vars.new_omega = -_v_att->yawspeed; // yaw speed is negative in the counter-clockwise direction

                //===============================================================================================================================
                // Throttle (throttle check separate to prevent sudden take off)
                _monocopter_vars.rot_err_p = (_params_thor.thor_rps_targ - _monocopter_vars.new_omega);  // yaw speed is negative in the counter-clockwise direction
                _monocopter_vars.rot_err_i = _monocopter_vars.rot_err_i + (_monocopter_vars.rot_err_p*delta_t);

                if (_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE] >= _params_thor.thor_thr_trig) {
                    collective_cmd = _params_thor.thor_coll_p * (_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE]- _params_thor.thor_thr_trig);
                    throttle_cmd = _params_thor.thor_rps_p * _monocopter_vars.rot_err_p + _params_thor.thor_rps_i * _monocopter_vars.rot_err_i;

                    if (throttle_cmd > 0.6f) {
                        throttle_cmd = 0.6f;
                    } else {
                        // Carry On
                    }
                } else {
                    _monocopter_vars.rot_err_i = 0;
                    collective_cmd = 0;
                    throttle_cmd = _actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];
                }

                //===============================================================================================================================
                // Direction (throttle check separate to prevent sudden take off)

                if (_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE] > _params_thor.thor_dir_trig) {
                    if (_monocopter_vars.new_psi < -M_PI_2_F && _monocopter_vars.old_psi > M_PI_2_F) {  // Mag passed the reset mark!
                        time_bef = _monocopter_vars.old_time + ((M_PI_F	 - _monocopter_vars.old_psi)/_monocopter_vars.new_omega);
                        time_aft = _monocopter_vars.new_time - ((M_PI_F	 + _monocopter_vars.new_psi)/_monocopter_vars.new_omega);
                        _monocopter_vars.loop_time = (time_bef + time_aft)/2.0f;
                    } else {
                        // Carry On
                    }

                    _monocopter_cmds.gymag_heading = -M_PI_F + (_monocopter_vars.new_omega * (_monocopter_vars.new_time - _monocopter_vars.loop_time));
                    _monocopter_cmds.loop_time     = _monocopter_vars.loop_time;

                    heading = _monocopter_cmds.gymag_heading;
                } else {
                    heading = -euler.psi();
                }

                des_dir   = atan2f(-_manual_control_sp->y, _manual_control_sp->x);
                des_mag   = _params_thor.thor_cyc_p*(sqrtf(powf(_manual_control_sp->y,2) + powf(_manual_control_sp->x,2)));
                cyclic_cmd  =  des_mag * cosf(heading - des_dir);

                //===============================================================================================================================
                // Persistence of Vision

                if ((heading < -1.3963f) && (heading > -1.7453f)) {
                    led_line =  1.0f;
                } else {
                    led_line = -1.0f;
                }

                // Send out commands
                _actuators_out_0->timestamp = _actuators_mc_in->timestamp;
                _actuators_out_0->control[0] = 0;
                _actuators_out_0->control[1] = 0;
                _actuators_out_0->control[2] = 0;
                _actuators_out_0->control[3] = throttle_cmd;
                _actuators_out_0->control[4] = 0;
                _actuators_out_0->control[5] = led_line;
                _actuators_out_0->control[6] = collective_cmd + cyclic_cmd;
                _actuators_out_0->control[7] = collective_cmd - cyclic_cmd;

                _actuators_out_1->timestamp = _actuators_mc_in->timestamp;
                _actuators_out_1->control[0] = 0;
                _actuators_out_1->control[1] = 0;
                _actuators_out_1->control[2] = 0;
                _actuators_out_1->control[3] = 0;
                _actuators_out_1->control[4] = 0;
                _actuators_out_1->control[5] = 0;
                _actuators_out_1->control[6] = 0;
                _actuators_out_1->control[7] = 0;

                // Update last time
                _monocopter_vars.old_time = hrt_absolute_time()/1000000.0f;
                _monocopter_vars.old_psi   = -euler.psi();

                //===============================================================================================================================
                // Publish the debugging data (to log currently)
                _monocopter_cmds.timestamp   = hrt_absolute_time();
                _monocopter_cmds.mag_heading = heading;
                _monocopter_cmds.des_dir = des_dir;
                _monocopter_cmds.des_mag = des_mag;
                _monocopter_cmds.rot_err_p = _monocopter_vars.rot_err_p;
                _monocopter_cmds.rot_err_i = _monocopter_vars.rot_err_i;

                // publish for debugging
                if (_monocopter_cmds_pub != nullptr) {
                        /* publish the attitude setpoint */
                        orb_publish(ORB_ID(monocopter_command), _monocopter_cmds_pub, &_monocopter_cmds);

                } else {
                        /* advertise and publish */
                        _monocopter_cmds_pub = orb_advertise(ORB_ID(monocopter_command), &_monocopter_cmds);
                }

            } else {
		_actuators_out_0->timestamp = _actuators_mc_in->timestamp;
		_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] = _actuators_mc_in->control[actuator_controls_s::INDEX_ROLL];
		_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH];
		_actuators_out_0->control[actuator_controls_s::INDEX_YAW] = _actuators_mc_in->control[actuator_controls_s::INDEX_YAW];
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];

		_actuators_out_1->timestamp = _actuators_mc_in->timestamp;

		if (_params->elevons_mc_lock) {
			_actuators_out_1->control[0] = 0;
			_actuators_out_1->control[1] = 0;

		} else {
			// NOTE: There is no mistake in the line below, multicopter yaw axis is controlled by elevon roll actuation!
			_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] =
				_actuators_mc_in->control[actuator_controls_s::INDEX_YAW];	//roll elevon
			_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
				_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH];	//pitch elevon
                }
            }

		break;

	case FIXED_WING:
		// in fixed wing mode we use engines only for providing thrust, no moments are generated
		_actuators_out_0->timestamp = _actuators_fw_in->timestamp;
		_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] = 0;
		_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] = 0;
		_actuators_out_0->control[actuator_controls_s::INDEX_YAW] = 0;
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];

		_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] =
			-_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL];	// roll elevon
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH];	// pitch elevon
		_actuators_out_1->control[actuator_controls_s::INDEX_YAW] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_YAW];	// yaw
		_actuators_out_1->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];	// throttle
		break;

	case TRANSITION_TO_FW:
	case TRANSITION_TO_MC:
		// in transition engines are mixed by weight (BACK TRANSITION ONLY)
		_actuators_out_0->timestamp = _actuators_mc_in->timestamp;
		_actuators_out_1->timestamp = _actuators_mc_in->timestamp;
		_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] = _actuators_mc_in->control[actuator_controls_s::INDEX_ROLL]
				* _mc_roll_weight;
		_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] * _mc_pitch_weight;
		_actuators_out_0->control[actuator_controls_s::INDEX_YAW] = _actuators_mc_in->control[actuator_controls_s::INDEX_YAW] *
				_mc_yaw_weight;
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];

		// NOTE: There is no mistake in the line below, multicopter yaw axis is controlled by elevon roll actuation!
		_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] = -_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL]
				* (1 - _mc_yaw_weight);
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] * _mc_pitch_weight;
		// **LATER** + (_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH] + _params->fw_pitch_trim) *(1 - _mc_pitch_weight);
		_actuators_out_1->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];
		break;
	}
}
