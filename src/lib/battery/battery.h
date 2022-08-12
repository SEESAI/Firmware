/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
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
 * @file battery.h
 *
 * Library calls for battery functionality.
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Timothy Scott <timothy@auterion.com>
 */

#pragma once

#include <math.h>
#include <float.h>

#include <board_config.h>
#include <px4_platform_common/board_common.h>
#include <px4_platform_common/module_params.h>
#include <matrix/math.hpp>

#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <lib/ecl/AlphaFilter/AlphaFilter.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_control_mode.h>

#include <systemlib/mavlink_log.h>

/**
 * BatteryBase is a base class for any type of battery.
 *
 * You can use this class on its own. Or, if you need to implement a custom battery type,
 * you can inherit from this class. See, for example, src/modules/battery_status/AnalogBattery.h
 */
class Battery : public ModuleParams
{
public:
	Battery(int index, ModuleParams *parent, const int sample_interval_us);
	~Battery() = default;

	/**
	 * Get the battery cell count
	 */
	int cell_count() { return _params.n_cells; }

	/**
	 * Get the empty voltage per cell
	 */
	float empty_cell_voltage() { return _params.v_empty; }

	/**
	 * Get the full voltage per cell
	 */
	float full_cell_voltage() { return _params.v_charged; }

	/**
	 * Update current battery status message.
	 *
	 * @param voltage_raw: Battery voltage, in Volts
	 * @param current_raw: Battery current, in Amps
	 * @param timestamp: Time at which the ADC was read (use hrt_absolute_time())
	 * @param source: Source type in relation to BAT%d_SOURCE param.
	 * @param priority: The brick number -1. The term priority refers to the Vn connection on the LTC4417
	 * @param throttle_normalized: Throttle of the vehicle, between 0 and 1
	 */
	void updateBatteryStatus(const hrt_abstime &timestamp, float voltage_v, float current_a, bool connected,
				 int source, int priority, float throttle_normalized);

protected:
	struct {
		param_t v_empty;
		param_t v_charged;
		param_t n_cells;
		param_t capacity;
		param_t v_load_drop;
		param_t r_internal;
		param_t low_thr;
		param_t crit_thr;
		param_t emergen_thr;
		param_t source;

		// TODO: These parameters are depracated. They can be removed entirely once the
		//  new version of Firmware has been around for long enough.
		param_t v_empty_old;
		param_t v_charged_old;
		param_t n_cells_old;
		param_t capacity_old;
		param_t v_load_drop_old;
		param_t r_internal_old;
		param_t source_old;
	} _param_handles{};

	struct {
		float v_empty;
		float v_charged;
		int32_t  n_cells;
		float capacity;
		float v_load_drop;
		float r_internal;
		float low_thr;
		float crit_thr;
		float emergen_thr;
		int32_t source;

		// TODO: These parameters are depracated. They can be removed entirely once the
		//  new version of Firmware has been around for long enough.
		float v_empty_old;
		float v_charged_old;
		int32_t  n_cells_old;
		float capacity_old;
		float v_load_drop_old;
		float r_internal_old;
		int32_t source_old;
	} _params{};

	const int _index;

	bool _first_parameter_update{true};
	void updateParams() override;

	/**
	 * This function helps migrating and syncing from/to deprecated parameters. BAT_* BAT1_*
	 * @tparam T Type of the parameter (int or float)
	 * @param old_param Handle to the old deprecated parameter (for example, param_find("BAT_N_CELLS"))
	 * @param new_param Handle to the new replacement parameter (for example, param_find("BAT1_N_CELLS"))
	 * @param old_val Pointer to the value of the old deprecated parameter
	 * @param new_val Pointer to the value of the new replacement parameter
	 * @param firstcall If true, this function prefers migrating old to new
	 */
	template<typename T>
	void migrateParam(param_t old_param, param_t new_param, T *old_val, T *new_val, bool firstcall)
	{
		T previous_old_val = *old_val;
		T previous_new_val = *new_val;

		// Update both the old and new parameter values
		param_get(old_param, old_val);
		param_get(new_param, new_val);

		// Check if the parameter values are different
		if (!matrix::isEqualF((float)*old_val, (float)*new_val)) {
			// If so, copy the new value over to the unchanged parameter
			// Note: If they differ from the beginning we migrate old to new
			if (firstcall || !matrix::isEqualF((float)*old_val, (float)previous_old_val)) {
				param_set_no_notification(new_param, old_val);
				param_get(new_param, new_val);

			} else if (!matrix::isEqualF((float)*new_val, (float)previous_new_val)) {
				param_set_no_notification(old_param, new_val);
				param_get(old_param, old_val);
			}
		}
	}

private:
	void sumDischarged(const hrt_abstime &timestamp, float current_a);
	void estimateStateOfCharge(const float voltage_v, const float current_a, const float throttle);
	uint8_t determineWarning(float state_of_charge);
	void computeScale();

	uORB::PublicationMulti<battery_status_s> _battery_status_pub{ORB_ID(battery_status)};

	bool _battery_initialized{false};
	AlphaFilter<float> _voltage_filter_v;
	AlphaFilter<float> _current_filter_a;
	AlphaFilter<float> _throttle_filter;
	float _discharged_mah{0.f};
	float _discharged_mah_loop{0.f};
	float _state_of_charge_volt_based{-1.f};	// [0,1]
	float _state_of_charge{-1.f};				// [0,1]
	float _scale{1.f};
	uint8_t _warning{battery_status_s::BATTERY_WARNING_NONE};
	hrt_abstime _last_timestamp{0};

	// Sees SOC calculation - a lookup table vs OC voltage at low current, and then pure coulomb coutning at high current
	float _soc_initial{0}; /// SOC at last low-current time (initialisation point for columnb counting)
	float _discharged_mah_initial{0}; /// discharged mah at last low-current time (initialisation point for column counting)
	hrt_abstime _sees_warning_last{0}; /// Time we last warned about cell voltage - used to limit warning frequency
	orb_advert_t _mavlink_log_pub{nullptr}; /// pointer to the mavlink publisher for the warning
	bool _armed{false}; /// Whether the drone is armed (Read from vehicle control mode sub)
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};

	/**
	 * Self-contained simple lookup class for SOC given a voltage
	 *
	 * NB
	 *   - currently const (not reading from params) just to get going quickly
	 *   - Please ensure _lookup_size is correctly set otherwise will seg_fault
	 *   - Please ensure _lookup is monotonic decreasing in voltage and SOC
	 *   - We couldn't find a nuttx table which has a .size() method!  Suggestions welcome
	 */
	struct SOCLookup {
		// return soc [0, 1] given open circuit cell voltage
		float GetSOC(float voltage);

	private:
		struct Lookup {
			float _oc_voltage{0.f};
			float _soc{0.f};
		};
		/// size of lookup table - NB MUST be correct or risk of seg_fault / bad lookup
		const int _lookup_size = 11;
		/// lookup table - NB MUST be monontonic descending
		const Lookup _lookup[11] {
			{4.17, 100.0},
			{4.09, 89.5},
			{3.99, 79.1},
			{3.93, 68.6},
			{3.87, 58.2},
			{3.82, 47.7},
			{3.79, 37.3},
			{3.77, 26.8},
			{3.73, 16.4},
			{3.69, 5.9},
			{3.50, 0.0}
		};
	} _soc_lookup;
};
