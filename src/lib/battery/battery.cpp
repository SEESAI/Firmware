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
 * @file battery.cpp
 *
 * Library calls for battery functionality.
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Timothy Scott <timothy@auterion.com>
 */

#include "battery.h"
#include <mathlib/mathlib.h>
#include <cstring>
#include <px4_platform_common/defines.h>

using namespace time_literals;
using namespace matrix;

// Sees: Reference Chemistry Curve for Tattu 25Ah HV packs (TN23.27).
// Fraction of chemical charge left vs open circuit cell voltage. Must match SI2's BatterySOCEstimator.
static constexpr float VOLTAGE_CURVE[] {3.300f, 3.554f, 3.678f, 3.722f, 3.757f, 3.795f, 3.838f, 3.902f, 4.002f, 4.095f, 4.203f, 4.350f};
static constexpr float CHARGE_CURVE[] {0.000f, 0.026f, 0.076f, 0.175f, 0.265f, 0.377f, 0.476f, 0.578f, 0.680f, 0.779f, 0.879f, 1.000f};

Battery::Battery(int index, ModuleParams *parent, const int sample_interval_us, const uint8_t source) :
	ModuleParams(parent),
	_index(index < 1 || index > 9 ? 1 : index),
	_source(source)
{
	const float expected_filter_dt = static_cast<float>(sample_interval_us) / 1_s;
	_current_average_filter_a.setParameters(expected_filter_dt, 50.f);
	_ocv_filter_v.setParameters(expected_filter_dt, 1.f);
	_cell_voltage_filter_v.setParameters(expected_filter_dt, 1.f);
	_cell_terminal_voltage_filter_v.setParameters(expected_filter_dt, 1.f);

	if (index > 9 || index < 1) {
		PX4_ERR("Battery index must be between 1 and 9 (inclusive). Received %d. Defaulting to 1.", index);
	}

	// 16 chars for parameter name + null terminator
	char param_name[17];

	snprintf(param_name, sizeof(param_name), "BAT%d_V_EMPTY", _index);
	_param_handles.v_empty = param_find(param_name);

	if (_param_handles.v_empty == PARAM_INVALID) {
		PX4_ERR("Could not find parameter with name %s", param_name);
	}

	snprintf(param_name, sizeof(param_name), "BAT%d_V_CHARGED", _index);
	_param_handles.v_charged = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_N_CELLS", _index);
	_param_handles.n_cells = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_CAPACITY", _index);
	_param_handles.capacity = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_R_INTERNAL", _index);
	_param_handles.r_internal = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_SOURCE", _index);
	_param_handles.source = param_find(param_name);

	_param_handles.low_thr = param_find("BAT_LOW_THR");
	_param_handles.crit_v = param_find("BAT_CRIT_V");
	_param_handles.crit_thr = param_find("BAT_CRIT_THR");
	_param_handles.emergen_thr = param_find("BAT_EMERGEN_THR");

	_param_handles.bat_avrg_current = param_find("BAT_AVRG_CURRENT");

	updateParams();
}

void Battery::updateVoltage(const float voltage_v)
{
	_voltage_v = voltage_v;
}

void Battery::updateCurrent(const float current_a)
{
	_current_a = current_a;
}

void Battery::updateTemperature(const float temperature_c)
{
	_temperature_c = temperature_c;
}

void Battery::updateBatteryStatus(const hrt_abstime &timestamp)
{
	updateDt(timestamp);

	// Require minimum voltage otherwise override connected status
	if (_voltage_v < LITHIUM_BATTERY_RECOGNITION_VOLTAGE) {
		_connected = false;
	}

	if (!_connected || (_last_unconnected_timestamp == 0)) {
		_last_unconnected_timestamp = timestamp;
	}

	// Wait with initializing filters to avoid relying on a voltage sample from the rising edge
	_battery_initialized = _connected && (timestamp > _last_unconnected_timestamp + 2_s);

	if (_connected && !_battery_initialized && _internal_resistance_initialized && _params.n_cells > 0) {
		resetInternalResistanceEstimation(_voltage_v, _current_a);
	}

	sumDischarged(_current_a);
	_state_of_charge_volt_based =
		calculateStateOfChargeVoltageBased(_voltage_v, _current_a);

	if (!_external_state_of_charge) {
		estimateStateOfCharge();
	}

	computeScale();

	if (_connected && _battery_initialized) {
		_warning = determineWarning(_state_of_charge);
	}
}

battery_status_s Battery::getBatteryStatus()
{
	battery_status_s battery_status{};
	battery_status.voltage_v = _voltage_v;
	battery_status.current_a = _current_a;
	battery_status.current_average_a = _current_average_filter_a.getState();
	battery_status.discharged_mah = _discharged_mah;
	battery_status.remaining = _state_of_charge;
	battery_status.scale = _scale;
	battery_status.time_remaining_s = computeRemainingTime(_current_a);
	battery_status.temperature = _temperature_c;
	battery_status.cell_count = _params.n_cells;
	battery_status.connected = _connected;
	battery_status.source = _source;
	battery_status.priority = _priority;
	battery_status.capacity = static_cast<uint16_t>(_capacity_mah);
	battery_status.id = static_cast<uint8_t>(_index);
	battery_status.warning = _warning;
	battery_status.timestamp = hrt_absolute_time();
	battery_status.faults = determineFaults();
	battery_status.internal_resistance_estimate = _internal_resistance_estimate;
	battery_status.ocv_estimate = _voltage_v + _internal_resistance_estimate * _params.n_cells * _current_a;
	battery_status.ocv_estimate_filtered = _ocv_filter_v.getState();
	battery_status.volt_based_soc_estimate = _params.n_cells > 0 ? math::interpolate(_ocv_filter_v.getState() /
			_params.n_cells,
			_params.v_empty, _params.v_charged, 0.f, 1.f) : -1.f;
	battery_status.voltage_prediction = _voltage_prediction;
	battery_status.prediction_error = _prediction_error;
	battery_status.estimation_covariance_norm = _estimation_covariance_norm;
	return battery_status;
}

void Battery::publishBatteryStatus(const battery_status_s &battery_status)
{
	if (_source == _params.source) {
		_battery_status_pub.publish(battery_status);
	}
}

void Battery::updateAndPublishBatteryStatus(const hrt_abstime &timestamp)
{
	updateBatteryStatus(timestamp);
	publishBatteryStatus(getBatteryStatus());
}
void Battery::updateDt(const hrt_abstime &timestamp)
{
	if (_last_timestamp != 0) {
		_dt = math::min((timestamp - _last_timestamp) / 1e6f, 2.f); // guard to a maximum 2 seconds dt
	}

	_last_timestamp = timestamp;
}

float Battery::sumDischarged(float current_a)
{
	if (_dt > FLT_EPSILON) {
		// mAh since last loop: (current[A] * 1000 = [mA]) * (dt[s] / 3600 = [h])
		_discharged_mah_loop = (current_a * 1e3f) * (_dt / 3600.f);
		_discharged_mah += _discharged_mah_loop;
	}

	return _discharged_mah;
}

float Battery::calculateStateOfChargeVoltageBased(const float voltage_v, const float current_a)
{
	if (_params.n_cells == 0) {
		return -1.0f;
	}

	// remaining battery capacity based on voltage
	float cell_voltage = voltage_v / _params.n_cells;
	_cell_terminal_voltage_filter_v.update(cell_voltage);

	// correct battery voltage locally for load drop according to internal resistance and current
	if (current_a > FLT_EPSILON) {
		updateInternalResistanceEstimation(voltage_v, current_a);

		if (_params.r_internal >= 0.f) { // Use user specified internal resistance value
			cell_voltage += _params.r_internal * current_a;

		} else { // Use estimated internal resistance value
			cell_voltage += _internal_resistance_estimate * current_a;
		}

	}

	_cell_voltage_filter_v.update(cell_voltage);
	return stateOfChargeFromVoltage(_cell_voltage_filter_v.getState());
}

float Battery::stateOfChargeFromVoltage(float cell_voltage)
{
	const float charge = math::interpolateNXY(cell_voltage, VOLTAGE_CURVE, CHARGE_CURVE);
	const float charge_empty = math::interpolateNXY(_params.v_empty, VOLTAGE_CURVE, CHARGE_CURVE);
	const float charge_charged = math::interpolateNXY(_params.v_charged, VOLTAGE_CURVE, CHARGE_CURVE);
	return math::interpolate(charge, charge_empty, charge_charged, 0.f, 1.f);
}

void Battery::updateInternalResistanceEstimation(const float voltage_v, const float current_a)
{
	Vector2f x{1, -current_a};
	_voltage_prediction = (x.transpose() * _RLS_est)(0, 0);
	_prediction_error = voltage_v - _voltage_prediction;
	const Vector2f gamma = _estimation_covariance * x / (LAMBDA + (x.transpose() * _estimation_covariance * x)(0, 0));
	const Vector2f RSL_est_temp = _RLS_est + gamma * _prediction_error;
	const Matrix2f estimation_covariance_temp = (_estimation_covariance
			- Matrix<float, 2, 1>(gamma) * (x.transpose() * _estimation_covariance)) / LAMBDA;
	const float estimation_covariance_temp_norm =
		sqrtf(powf(estimation_covariance_temp(0, 0), 2.f)
		      + 2.f * powf(estimation_covariance_temp(1, 0), 2.f)
		      + powf(estimation_covariance_temp(1, 1), 2.f));

	if (estimation_covariance_temp_norm < _estimation_covariance_norm) { // Only update if estimation improves
		_RLS_est = RSL_est_temp;
		_estimation_covariance = estimation_covariance_temp;
		_estimation_covariance_norm = estimation_covariance_temp_norm;
		_internal_resistance_estimate =
			math::max(_RLS_est(1) / _params.n_cells, 0.f); // Only use positive values

	} else { // Update OCV estimate with IR estimate
		_RLS_est(0) = voltage_v + _RLS_est(1) * current_a;
	}

	_ocv_filter_v.update(voltage_v + _internal_resistance_estimate * _params.n_cells * current_a);
}

void Battery::resetInternalResistanceEstimation(const float voltage_v, const float current_a)
{
	_RLS_est(0) = voltage_v;
	_RLS_est(1) = R_DEFAULT * _params.n_cells;
	_estimation_covariance.setZero();
	_estimation_covariance(0, 0) = OCV_COVARIANCE * _params.n_cells;
	_estimation_covariance(1, 1) = R_COVARIANCE * _params.n_cells;
	_estimation_covariance_norm = sqrtf(powf(_estimation_covariance(0, 0), 2.f) + 2.f * powf(_estimation_covariance(1, 0),
					    2.f) + powf(_estimation_covariance(1, 1), 2.f));
	_internal_resistance_estimate = R_DEFAULT;
	_ocv_filter_v.reset(voltage_v + _internal_resistance_estimate * _params.n_cells * current_a);
	_cell_terminal_voltage_filter_v.reset(voltage_v / _params.n_cells);

	if (_params.r_internal >= 0.f) { // Use user specified internal resistance value
		_cell_voltage_filter_v.reset(voltage_v / _params.n_cells + _params.r_internal * current_a);

	} else { // Use estimated internal resistance value
		_cell_voltage_filter_v.reset(voltage_v / _params.n_cells + _internal_resistance_estimate * current_a);
	}
}

void Battery::estimateStateOfCharge()
{
	// Sees: SoC from voltage while disarmed (current ~0), then count the charge used while armed.
	// Negative current is ignored, as in v1.13 and SI2. See docs/sees/30_battery_state_of_charge.md
	if (_armed && _battery_initialized && _capacity_mah > 0.f) {
		const float used_mah = math::max(_discharged_mah_loop, 0.f);
		_state_of_charge = math::constrain(_state_of_charge - used_mah / _capacity_mah, 0.f, 1.f);

	} else {
		_state_of_charge = _state_of_charge_volt_based;
	}
}

uint8_t Battery::determineWarning(float state_of_charge)
{
	if (state_of_charge < _params.emergen_thr) {
		return battery_status_s::WARNING_EMERGENCY;

	} else if (state_of_charge < _params.crit_thr) {
		return battery_status_s::WARNING_CRITICAL;

	} else if (state_of_charge < _params.low_thr) {
		return battery_status_s::WARNING_LOW;

	} else {
		return battery_status_s::WARNING_NONE;
	}
}

uint16_t Battery::determineFaults()
{
	uint16_t faults{0};

	if ((_params.n_cells > 0)
	    && (_voltage_v > (_params.n_cells * _params.v_charged * 1.05f))) {
		// Reported as a "spike" since "over-voltage" does not exist in MAV_BATTERY_FAULT
		faults |= (1 << battery_status_s::FAULT_SPIKES);
	}

	// Sees: nothing corrects SoC in flight, so flag low measured (not load-compensated) cell voltage
	if (_battery_initialized && (_params.n_cells > 0) && (_cell_terminal_voltage_filter_v.getState() < _params.crit_v)) {
		faults |= (1 << battery_status_s::FAULT_DEEP_DISCHARGE);
	}

	return faults;
}

void Battery::computeScale()
{
	_scale = _params.v_charged / _cell_voltage_filter_v.getState();

	if (PX4_ISFINITE(_scale)) {
		_scale = math::constrain(_scale, 1.f, 1.3f); // Allow at most 30% compensation

	} else {
		_scale = 1.f;
	}
}

float Battery::computeRemainingTime(float current_a)
{
	float time_remaining_s = NAN;
	bool reset_current_avg_filter = false;

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

			if (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING && !_vehicle_status_is_fw) {
				reset_current_avg_filter = true;
			}

			_vehicle_status_is_fw = (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
		}
	}

	_flight_phase_estimation_sub.update();

	// reset filter if not feasible, negative or we did a VTOL transition to FW mode
	if (!PX4_ISFINITE(_current_average_filter_a.getState()) || _current_average_filter_a.getState() < FLT_EPSILON
	    || reset_current_avg_filter) {
		_current_average_filter_a.reset(_params.bat_avrg_current);
	}

	if (_armed && PX4_ISFINITE(current_a)) {
		// For FW only update when we are in level flight
		if (!_vehicle_status_is_fw || ((hrt_absolute_time() - _flight_phase_estimation_sub.get().timestamp) < 2_s
					       && _flight_phase_estimation_sub.get().flight_phase == flight_phase_estimation_s::FLIGHT_PHASE_LEVEL)) {
			if (_dt > FLT_EPSILON) {
				_current_average_filter_a.update(fmaxf(current_a, 0.f), _dt);

			} else {
				_current_average_filter_a.update(fmaxf(current_a, 0.f));
			}
		}
	}

	// Remaining time estimation only possible with capacity
	if (_capacity_mah > 0.f) {
		const float remaining_capacity_mah = _state_of_charge * _capacity_mah;
		const float current_ma = fmaxf(_current_average_filter_a.getState() * 1e3f, FLT_EPSILON);
		time_remaining_s = remaining_capacity_mah / current_ma * 3600.f;
	}

	return time_remaining_s;
}

void Battery::updateParams()
{
	const int n_cells = _first_parameter_update ? 0 : _params.n_cells;
	param_get(_param_handles.v_empty, &_params.v_empty);
	param_get(_param_handles.v_charged, &_params.v_charged);
	param_get(_param_handles.n_cells, &_params.n_cells);
	param_get(_param_handles.r_internal, &_params.r_internal);
	param_get(_param_handles.source, &_params.source);
	param_get(_param_handles.low_thr, &_params.low_thr);
	param_get(_param_handles.crit_v, &_params.crit_v);
	param_get(_param_handles.crit_thr, &_params.crit_thr);
	param_get(_param_handles.emergen_thr, &_params.emergen_thr);
	param_get(_param_handles.bat_avrg_current, &_params.bat_avrg_current);

	float capacity{0.f};
	param_get(_param_handles.capacity, &capacity);
	setCapacityMah(capacity);

	if (n_cells != _params.n_cells) {
		_internal_resistance_initialized = false;
	}

	if (!_internal_resistance_initialized && _params.n_cells > 0) {
		_RLS_est(0) = OCV_DEFAULT * _params.n_cells;
		_RLS_est(1) = R_DEFAULT * _params.n_cells;
		_estimation_covariance(0, 0) = OCV_COVARIANCE * _params.n_cells;
		_estimation_covariance(0, 1) = 0.f;
		_estimation_covariance(1, 0) = 0.f;
		_estimation_covariance(1, 1) = R_COVARIANCE * _params.n_cells;
		_estimation_covariance_norm = sqrtf(powf(_estimation_covariance(0, 0), 2.f) + 2.f * powf(_estimation_covariance(1, 0),
						    2.f) + powf(_estimation_covariance(1, 1), 2.f));
		_internal_resistance_initialized = true;
	}

	ModuleParams::updateParams();

	_first_parameter_update = false;
}
