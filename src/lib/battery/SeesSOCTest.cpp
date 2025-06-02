#include <gtest/gtest.h>
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
	}

	void TearDown() override
	{
		delete battery;
	}

protected:
	TestableBattery *battery;
};

// Test that the SOC calculation returns 100% (1.0) when given the maximum voltage (4.35V)
// This validates that a fully charged battery is correctly identified
TEST_F(SeesSOCTest, TestGetSOCAtMaxVoltage)
{
	float result = battery->testGetSOC(4.35f);
	EXPECT_NEAR(result, 1.0f, 0.01f);
}

// Test that the SOC calculation returns 0% (0.0) when given the minimum voltage (3.4V)
// This validates that an empty battery is correctly identified
TEST_F(SeesSOCTest, TestGetSOCAtMinVoltage)
{
	float result = battery->testGetSOC(3.4f);
	EXPECT_NEAR(result, 0.0f, 0.01f);
}

// Test that SOC values increase monotonically with increasing voltage
// This ensures the SOC calculation behaves predictably across the voltage range
TEST_F(SeesSOCTest, TestGetSOCMonotonicBehavior)
{
	std::vector<float> test_voltages = {3.5f, 3.7f, 3.9f, 4.1f, 4.3f};
	std::vector<float> soc_values;

	for (float voltage : test_voltages) {
		soc_values.push_back(battery->testGetSOC(voltage));
	}

	for (size_t i = 1; i < soc_values.size(); i++) {
		EXPECT_GT(soc_values[i], soc_values[i - 1])
				<< "SOC should increase with voltage";
	}
}

// Test that voltages outside the normal range are properly clamped
// Voltages above max should return 100%, voltages below min should return 0%
TEST_F(SeesSOCTest, TestGetSOCBoundaryConditions)
{
	float result_above_max = battery->testGetSOC(5.0f);
	float result_below_min = battery->testGetSOC(3.0f);

	EXPECT_NEAR(result_above_max, 1.0f, 0.01f);
	EXPECT_NEAR(result_below_min, 0.0f, 0.01f);
}

// Test that SOC values are always within the valid range [0.0, 1.0]
// This ensures no calculation errors produce invalid SOC values
TEST_F(SeesSOCTest, TestSOCRange)
{
	std::vector<float> test_voltages = {3.0f, 3.5f, 3.9f, 4.1f, 4.35f, 5.0f};

	for (float voltage : test_voltages) {
		float soc = battery->testGetSOC(voltage);
		std::cout << "Voltage: " << voltage << "V, SOC: " << soc << std::endl;
		EXPECT_GE(soc, 0.0f) << "SOC should be >= 0.0 for voltage " << voltage;
		EXPECT_LE(soc, 1.0f) << "SOC should be <= 1.0 for voltage " << voltage;
	}
}


// Test that battery parameter values are correctly configured
// This validates the test setup and parameter initialization
TEST_F(SeesSOCTest, TestParameterIntegration)
{
	EXPECT_EQ(battery->cell_count(), 6);
	EXPECT_NEAR(battery->empty_cell_voltage(), 3.4f, 0.01f);
	EXPECT_NEAR(battery->full_cell_voltage(), 4.35f, 0.01f);
}

// Test SOC calculation at specific key voltages (empty, charged)
// This validates the normalization and scaling of SOC values
TEST_F(SeesSOCTest, TestSOCNormalization)
{
	float soc_empty = battery->testGetSOC(3.4f);
	float soc_charged = battery->testGetSOC(4.35f);

	EXPECT_NEAR(soc_empty, 0.0f, 0.01f);
	EXPECT_NEAR(soc_charged, 1.0f, 0.01f);
}

// Test that chemistry lookup values from the lookup table are reasonable
// This validates specific points in the non-linear chemistry curve
TEST_F(SeesSOCTest, TestChemistryLookupTableValues)
{
// Test known lookup table values (voltage -> expected chemistry SOC)
	float lookup_4_203 = battery->testChemistryLookup(4.203f);  // Should be around 0.88
	float lookup_3_838 = battery->testChemistryLookup(3.838f);  // Should be around 0.48
	float lookup_3_678 = battery->testChemistryLookup(3.678f);  // Should be around 0.08

	EXPECT_NEAR(lookup_4_203, 0.88f, 0.02f);
	EXPECT_NEAR(lookup_3_838, 0.48f, 0.02f);
	EXPECT_NEAR(lookup_3_678, 0.08f, 0.02f);
}
