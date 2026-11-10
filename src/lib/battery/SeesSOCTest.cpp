/**
 * Sees: property tests for the battery state of charge estimator. See docs/sees/30_battery_state_of_charge.md
 *
 *   disarmed: SoC = S(V)        armed: SoC = SoC - max(dQ, 0) / C, clamped to [0, 1]
 *
 * The properties hold for any window and threshold, so those are left at their defaults.
 */

#include <gtest/gtest.h>
#include <memory>
#include <uORB/Publication.hpp>
#include "battery.h"

using namespace time_literals;

static constexpr int32_t N_CELLS = 6;
static constexpr float CAPACITY_MAH = 10000.f;

class SeesSOCTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		// Enable voltage and coulomb counting; everything else keeps its default
		param_control_autosave(false);
		param_reset_all();
		setParam("BAT1_N_CELLS", N_CELLS);
		setParam("BAT1_CAPACITY", CAPACITY_MAH);
		param_get(param_find("BAT_CRIT_V"), &_crit_v);

		_battery = std::make_unique<Battery>(1, nullptr, 10_ms, battery_status_s::SOURCE_POWER_MODULE);
		_v_empty = _battery->empty_cell_voltage();
		_v_charged = _battery->full_cell_voltage();

		_battery->setConnected(true);
		disarm();
		hold((_v_empty + _v_charged) / 2.f, 0.f, 5_s); // past the 2 s connection delay
	}

	template<typename T>
	static void setParam(const char *name, T value) { param_set(param_find(name), &value); }

	void arm(bool armed = true)
	{
		vehicle_status_s vehicle_status{};
		vehicle_status.arming_state = armed ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_DISARMED;
		_vehicle_status_pub.publish(vehicle_status);
		_status = _battery->getBatteryStatus(); // the battery reads the arming state here
	}

	void disarm() { arm(false); }

	// Hold a constant cell voltage and pack current at 100 Hz, long enough for the 1 s filters to settle
	void hold(float cell_voltage, float current_a, hrt_abstime duration = 10_s)
	{
		for (const hrt_abstime end = _time + duration; _time < end; _time += 10_ms) {
			_battery->updateVoltage(N_CELLS * cell_voltage);
			_battery->updateCurrent(current_a);
			_battery->updateBatteryStatus(_time);
			_status = _battery->getBatteryStatus();
		}
	}

	float S(float cell_voltage) { return _battery->stateOfChargeFromVoltage(cell_voltage); }
	float soc() const { return _status.remaining; }
	float socPlusUsed() const { return _status.remaining + _status.discharged_mah / CAPACITY_MAH; }
	bool fault() const { return _status.faults & (1 << battery_status_s::FAULT_DEEP_DISCHARGE); }

	std::unique_ptr<Battery> _battery;
	battery_status_s _status{};
	float _v_empty{}, _v_charged{}, _crit_v{};
	hrt_abstime _time{1_s};
	uORB::Publication<vehicle_status_s> _vehicle_status_pub{ORB_ID(vehicle_status)};
};

// S rises from 0 at V_EMPTY to 1 at V_CHARGED, and stays in [0, 1] outside that window
TEST_F(SeesSOCTest, VoltageMapIsMonotoneOntoUnitInterval)
{
	EXPECT_FLOAT_EQ(S(_v_empty), 0.f);
	EXPECT_FLOAT_EQ(S(_v_charged), 1.f);

	for (float v = _v_empty - 0.3f; v <= _v_charged + 0.3f; v += 0.005f) {
		EXPECT_GE(S(v), 0.f) << v << " V";
		EXPECT_LE(S(v), 1.f) << v << " V";
	}

	for (float v = _v_empty; v < _v_charged; v += 0.005f) {
		EXPECT_LT(S(v), S(v + 0.005f)) << v << " V";
	}
}

// Disarmed, SoC is S(V), whatever happened while armed
TEST_F(SeesSOCTest, DisarmedSocIsVoltageMap)
{
	for (float v = _v_empty - 0.1f; v <= _v_charged + 0.1f; v += 0.05f) {
		arm();
		hold(v, 65.f);
		disarm();
		hold(v, 0.f);
		EXPECT_NEAR(soc(), S(v), 1e-4f) << v << " V";
	}
}

// Armed, charge is conserved: SoC + Q / C stays constant, whatever the voltage or current
TEST_F(SeesSOCTest, ArmedConservesCharge)
{
	arm();
	const float conserved = socPlusUsed();

	for (float current : {0.f, 20.f, 65.f, 150.f}) {
		for (float v = _v_empty; v <= _v_charged; v += 0.15f) {
			hold(v, current);
			EXPECT_NEAR(socPlusUsed(), conserved, 1e-4f) << current << " A, " << v << " V";
		}
	}
}

// Armed, SoC never rises (negative current is ignored) and never drops below 0
TEST_F(SeesSOCTest, ArmedSocNeverRisesNorGoesNegative)
{
	arm();
	const float start = soc();

	hold(_v_charged, -2000.f, 60_s);
	EXPECT_FLOAT_EQ(soc(), start);

	hold(_v_charged, 2000.f, 60_s);
	EXPECT_FLOAT_EQ(soc(), 0.f);

	hold(_v_charged, -2000.f, 60_s);
	EXPECT_FLOAT_EQ(soc(), 0.f);
}

// The fault is set exactly when the measured (not load-compensated) cell voltage is below BAT_CRIT_V, at any current
TEST_F(SeesSOCTest, FaultIffMeasuredVoltageBelowCritV)
{
	for (float current : {0.f, 65.f}) {
		for (float dv : {-0.1f, -0.01f, 0.01f, 0.1f}) {
			hold(_crit_v + dv, current);
			EXPECT_EQ(fault(), dv < 0.f) << current << " A, " << dv << " V from BAT_CRIT_V";
		}
	}
}
