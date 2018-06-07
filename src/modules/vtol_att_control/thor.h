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
* @file thor.h
*
* @author Low Jun En 	<lowjunen@gmail.com>
*
*/

#ifndef THOR_H
#define THOR_H

#include "vtol_type.h"
#include <systemlib/perf_counter.h>  /** is it necsacery? **/
#include <systemlib/param/param.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/monocopter_command.h>

class Thor : public VtolType
{

public:
	Thor(VtolAttitudeControl *_att_controller);
	~Thor();

	virtual void update_vtol_state();
	virtual void update_transition_state();
	virtual void update_mc_state();
	virtual void update_fw_state();
	virtual void fill_actuator_outputs();
	virtual void waiting_on_tecs();

private:

        // handlers for publishers
        orb_advert_t	_monocopter_cmds_pub{nullptr};	// publisher for monocopter command debugging

        // data containers
        monocopter_command_s           _monocopter_cmds{};

	struct {
		float front_trans_dur_p2;
                float thor_rps_targ;
                float thor_rps_p;
                float thor_rps_i;
                float thor_coll_p;
                float thor_cyc_p;
                float thor_thr_trig;
                float thor_dir_trig;
        } _params_thor;

	struct {
		param_t front_trans_dur_p2;
                param_t thor_rps_targ;
                param_t thor_rps_p;
                param_t thor_rps_i;
                param_t thor_coll_p;
                param_t thor_cyc_p;
                param_t thor_thr_trig;
                param_t thor_dir_trig;
        } _params_handles_thor;

	enum vtol_mode {
		MC_MODE = 0,			/**< vtol is in multicopter mode */
                TRANSITION_FRONT_P1,            /**< vtol is in front transition part 1 mode */
		TRANSITION_BACK,		/**< vtol is in back transition mode */
                FW_MODE				/**< vtol is in fixed wing mode */
	};

	struct {
                vtol_mode flight_mode;		/**< vtol flight mode, defined by enum vtol_mode */
		hrt_abstime transition_start;	/**< absoulte time at which front transition started */
	} _vtol_schedule;

        struct {
            float new_time;
            float old_time;
            float loop_time;

            float rot_err_p;
            float rot_err_i;

            float new_omega;
            float new_psi;
            float old_psi;
        } _monocopter_vars;

	float _thrust_transition_start; // throttle value when we start the front transition
	float _yaw_transition;	// yaw angle in which transition will take place
	float _pitch_transition_start;  // pitch angle at the start of transition (tailsitter)

	/**
	 * Update parameters.
	 */
	virtual void parameters_update();

};
#endif
