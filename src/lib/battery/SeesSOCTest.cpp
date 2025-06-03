#include <gtest/gtest.h>
#include <iostream>
#include "battery.h"

// Minimal ModuleParams for testing
class TestModuleParams : public ModuleParams
{
public:
	TestModuleParams() : ModuleParams(nullptr) {}
};

// Simplified test battery that directly sets parameter values
class TestableBattery : public Battery
{
public:
	TestableBattery() : Battery(1, &test_parent, 100000, 0)
	{
		// Directly set test parameter values
		_params.v_empty = 3.4f;
		_params.v_charged = 4.35f;
		_params.n_cells = 6;
		_params.capacity = 32000.0f;
		_params.r_internal = 0.01f;
		_params.low_thr = 0.2f;
		_params.crit_v = 3.4f;
		_params.crit_thr = 0.1f;
		_params.emergen_thr = 0.05f;
		_params.source = 0;
		_params.bat_avrg_current = 5.0f;
		_params.v_load_drop = 0.1f;
	}

	// Expose SeesSOC for testing
	float testGetSOC(float oc_voltage)
	{
		return _sees_soc.GetSOC(oc_voltage);
	}

	float testChemistryLookup(float voltage)
	{
		return _sees_soc.ChemistryLookup(voltage);
	}

private:
	TestModuleParams test_parent;
};

class SeesSOCTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		battery = new TestableBattery();

		// Validate test setup parameters
		EXPECT_EQ(battery->cell_count(), 6);
		EXPECT_NEAR(battery->empty_cell_voltage(), 3.4f, 0.01f);
		EXPECT_NEAR(battery->full_cell_voltage(), 4.35f, 0.01f);
	}

	void TearDown() override
	{
		delete battery;
	}

protected:
	TestableBattery *battery;
};

// Test SOC calculation at boundary voltages (empty=0%, charged=100%)
// Also validates that out-of-range voltages are properly clamped
TEST_F(SeesSOCTest, TestSOCBoundariesAndClamping)
{
// Test operational boundaries
	EXPECT_NEAR(battery->testGetSOC(3.4f), 0.0f, 0.01f) << "Empty voltage should return 0% SOC";
	EXPECT_NEAR(battery->testGetSOC(4.35f), 1.0f, 0.01f) << "Charged voltage should return 100% SOC";

// Test clamping behavior for out-of-range voltages
	EXPECT_NEAR(battery->testGetSOC(3.0f), 0.0f, 0.01f) << "Below-empty voltage should be clamped to 0%";
	EXPECT_NEAR(battery->testGetSOC(5.0f), 1.0f, 0.01f) << "Above-charged voltage should be clamped to 100%";
}

// Test that SOC values increase monotonically with increasing voltage
TEST_F(SeesSOCTest, TestSOCMonotonicBehavior)
{
	std::vector<float> test_voltages = {3.5f, 3.7f, 3.9f, 4.1f, 4.3f};
	std::vector<float> soc_values;

	for (float voltage : test_voltages) {
		soc_values.push_back(battery->testGetSOC(voltage));
	}

	for (size_t i = 1; i < soc_values.size(); i++) {
		EXPECT_GT(soc_values[i], soc_values[i - 1])
				<< "SOC should increase monotonically with voltage";
	}
}

// Test that all SOC values are within valid range [0.0, 1.0] across voltage spectrum
TEST_F(SeesSOCTest, TestSOCValidRange)
{
	std::vector<float> test_voltages = {3.0f, 3.5f, 3.9f, 4.1f, 4.35f, 5.0f};

	for (float voltage : test_voltages) {
		float soc = battery->testGetSOC(voltage);
		EXPECT_GE(soc, 0.0f) << "SOC should be >= 0.0 for voltage " << voltage;
		EXPECT_LE(soc, 1.0f) << "SOC should be <= 1.0 for voltage " << voltage;
	}
}

// Test chemistry lookup table interpolation at known data points
TEST_F(SeesSOCTest, TestChemistryLookupAccuracy)
{
// Test known lookup table values to ensure proper interpolation
	EXPECT_NEAR(battery->testChemistryLookup(4.203f), 0.88f, 0.02f) << "4.203V should map to ~88% chemistry SOC";
	EXPECT_NEAR(battery->testChemistryLookup(3.838f), 0.48f, 0.02f) << "3.838V should map to ~48% chemistry SOC";
	EXPECT_NEAR(battery->testChemistryLookup(3.678f), 0.08f, 0.02f) << "3.678V should map to ~8% chemistry SOC";
}
