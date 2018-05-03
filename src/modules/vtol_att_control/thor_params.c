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
 * @file thor_params.c
 * Parameters for thor attitude controller.
 *
 * @author Low Jun En <lowjunen@gmail.com>
 */

/**
 * Monocopter Target RPS
 *
 * @min 0.0
 * @max 100.0
 * @increment 0.50
 * @decimal 3
 * @group Monocopter Control
 */
PARAM_DEFINE_FLOAT(VT_THOR_RPS_TARG, 0.0f);

/**
 * Monocopter RPS P Gain
 *
 * @min 0.0
 * @max 10.0
 * @increment 0.01
 * @decimal 3
 * @group Monocopter Control
 */
PARAM_DEFINE_FLOAT(VT_THOR_RPS_P, 0.0f);

/**
 * Monocopter RPS I Gain
 *
 * @min 0.0
 * @max 10.0
 * @increment 0.01
 * @decimal 3
 * @group Monocopter Control
 */
PARAM_DEFINE_FLOAT(VT_THOR_RPS_I, 0.0f);

/**
 * Monocopter Collective P Gain
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group Monocopter Control
 */
PARAM_DEFINE_FLOAT(VT_THOR_COLL_P, 0.0f);


