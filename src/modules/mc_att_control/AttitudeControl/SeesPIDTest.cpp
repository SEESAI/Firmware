/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
 * Property tests of the attitude setpoint rate feed-forward, K_ff * F * s * theta_T in (5) of
 * docs/sees/121_rate_controller.md. Each test checks one property over random cases.
 */

#include <gtest/gtest.h>
#include <AttitudeControl.hpp>
#include <random>

using namespace matrix;

static constexpr int kCases = 20;
static constexpr float kGain = 0.138f;
static constexpr float kCutoff = 20.f;
static constexpr float kTol = 1e-4f;

struct Random {
	std::mt19937 gen{26};
	float uniform(float lo, float hi) { return std::uniform_real_distribution<float>(lo, hi)(gen); }
	Vector3f vector(float max) { return Vector3f(uniform(-max, max), uniform(-max, max), uniform(-max, max)); }
	Quatf attitude() { return Quatf(AxisAnglef(vector(M_PI_F))); }
	float dt() { return uniform(0.001f, 0.05f); } // [s] between setpoints, 20 Hz to 1 kHz
};

static Vector3f rollPitch(const Vector3f &v) { return Vector3f(v(0), v(1), 0.f); }

// A random rate of a target at attitude q that only changes its tilt, i.e. with no part about the vertical
static Vector3f tiltRate(Random &random, const Quatf &q, float max)
{
	const Vector3f vertical = q.inversed().dcm_z();
	const Vector3f r = random.vector(max);
	return r - r.dot(vertical) * vertical;
}

// Attitude control with only the feed-forward, so update() returns the feed-forward
static AttitudeControl feedForwardOnly(float gain = kGain, float rate_limit = 100.f)
{
	AttitudeControl ac;
	ac.setProportionalGain(Vector3f(), 1.f);
	ac.setRateLimit(Vector3f(rate_limit, rate_limit, rate_limit));
	ac.setFeedForward(gain, kCutoff);
	return ac;
}

// A new setpoint, then the rate setpoint for the drone at attitude q
static Vector3f step(AttitudeControl &ac, const Quatf &target, const Quatf &q, float dt)
{
	ac.setAttitudeSetpoint(target, 0.f, dt);
	return ac.update(q);
}

// (5) only has s * theta_T, so a constant target gives zero, whatever the drone's attitude
TEST(SeesPIDTest, ConstantTargetGivesZero)
{
	Random random;

	for (int n = 0; n < kCases; n++) {
		AttitudeControl ac = feedForwardOnly();
		const Quatf target = random.attitude();
		const float dt = random.dt();

		for (int i = 0; i < 100; i++) {
			EXPECT_LT(step(ac, target, random.attitude(), dt).norm(), kTol);
		}
	}
}

// F has unit DC gain, so a target tilting at r settles to K_ff * r, in the body frame, roll and pitch only,
// at any setpoint rate
TEST(SeesPIDTest, SteadyRateGivesGainTimesRate)
{
	Random random;

	for (int n = 0; n < kCases; n++) {
		AttitudeControl ac = feedForwardOnly();
		const Quatf q0 = random.attitude();
		const Quatf offset = random.attitude(); // of the drone from the target
		const Vector3f r = tiltRate(random, q0, 1.f);
		const float dt = random.dt();
		Vector3f ff;

		for (float t = 0.f; t < 1.f; t += dt) {
			const Quatf target = q0 * Quatf(AxisAnglef(r * t));
			ff = step(ac, target, target * offset, dt);
		}

		EXPECT_LT((ff - kGain * rollPitch(offset.inversed().rotateVector(r))).norm(), kTol);
	}
}

// F preserves area, so a change delta in the target integrates to K_ff * delta, however it is spread over
// setpoints. Half the cases are one large step, which needs the full log map, not a small-angle approximation.
TEST(SeesPIDTest, StepIntegralIsGainTimesStep)
{
	Random random;

	for (int n = 0; n < kCases; n++) {
		AttitudeControl ac = feedForwardOnly();
		const Vector3f delta = rollPitch(random.vector(1.5f));
		const int ramp = (n % 2) ? 1 : 2 + n;
		const float dt = random.dt();
		Vector3f integral;
		step(ac, Quatf(), Quatf(), dt);

		for (int i = 1; i < 250; i++) {
			integral += step(ac, Quatf(AxisAnglef(delta * fminf(i / float(ramp), 1.f))), Quatf(), dt) * dt;
		}

		EXPECT_LT((integral - kGain * rollPitch(delta)).norm(), kTol);
	}
}

// F is first order, so after a step delta the feed-forward is K_ff * delta / dt * a * (1 - a)^k,
// with a = dt / (tau + dt)
TEST(SeesPIDTest, StepResponseIsFirstOrder)
{
	Random random;

	for (int n = 0; n < kCases; n++) {
		AttitudeControl ac = feedForwardOnly();
		const Vector3f delta = rollPitch(random.vector(0.3f));
		const float dt = random.dt();
		const float a = dt / (1.f / (M_TWOPI_F * kCutoff) + dt);
		Vector3f expected = kGain * delta / dt * a;
		step(ac, Quatf(), Quatf(), dt);

		for (int k = 0; k < 20; k++) {
			EXPECT_LT((step(ac, Quatf(AxisAnglef(delta)), Quatf(), dt) - expected).norm(), kTol * (1.f + expected.norm()));
			expected *= 1.f - a;
		}
	}
}

// q and -q are the same attitude, so flipping the setpoint quaternion's sign changes nothing
TEST(SeesPIDTest, QuaternionSignDoesNotMatter)
{
	Random random;

	for (int n = 0; n < kCases; n++) {
		AttitudeControl ac = feedForwardOnly(), ac_flipped = feedForwardOnly();
		const Quatf q0 = random.attitude();
		const Vector3f r = random.vector(1.f);
		const float dt = random.dt();

		for (int i = 0; i < 100; i++) {
			const Quatf target = q0 * Quatf(AxisAnglef(r * (i * dt)));
			const Quatf flipped = (random.uniform(0.f, 1.f) < 0.5f) ? Quatf(-target(0), -target(1), -target(2), -target(3)) : target;
			EXPECT_LT((step(ac_flipped, flipped, target, dt) - step(ac, target, target, dt)).norm(), kTol);
		}
	}
}

// A heading reset rotates the setpoint about the vertical without the target moving, so it gives zero,
// whether or not the setpoint was adapted before the next one arrives
TEST(SeesPIDTest, HeadingResetGivesZero)
{
	Random random;

	for (int n = 0; n < kCases; n++) {
		AttitudeControl ac = feedForwardOnly(), ac_adapted = feedForwardOnly();
		const Quatf target = random.attitude();
		const Quatf q = random.attitude();
		const Quatf reset(AxisAnglef(0.f, 0.f, random.uniform(-M_PI_F, M_PI_F)));
		const float dt = random.dt();
		step(ac, target, q, dt);
		step(ac_adapted, target, q, dt);
		ac_adapted.adaptAttitudeSetpoint(reset);
		EXPECT_LT(step(ac, reset * target, q, dt).norm(), kTol);
		EXPECT_LT(step(ac_adapted, reset * target, q, dt).norm(), kTol);
	}
}

// Yaw has its own feed-forward, so a tilted target yawing about the vertical gives zero
TEST(SeesPIDTest, YawGivesZero)
{
	Random random;

	for (int n = 0; n < kCases; n++) {
		AttitudeControl ac = feedForwardOnly();
		const Quatf q0 = random.attitude();
		const float yawspeed = random.uniform(-3.f, 3.f);
		const float dt = random.dt();

		for (int i = 0; i < 100; i++) {
			const Quatf target = Quatf(AxisAnglef(0.f, 0.f, yawspeed * i * dt)) * q0;
			EXPECT_LT(step(ac, target, target, dt).norm(), kTol);
		}
	}
}

// A non-finite setpoint does not stick in the feed-forward: like stock PX4, the next good setpoint recovers
TEST(SeesPIDTest, NonFiniteSetpointRecovers)
{
	Random random;

	for (int n = 0; n < kCases; n++) {
		AttitudeControl ac = feedForwardOnly();
		const Quatf target = random.attitude();
		const Quatf q = random.attitude();
		const float dt = random.dt();
		step(ac, target, q, dt);
		step(ac, Quatf(NAN, NAN, NAN, NAN), q, dt);
		EXPECT_LT(step(ac, target, q, dt).norm(), kTol);
	}
}

// With K_ff = 0 the rate setpoint is exactly stock PX4, for any inputs
TEST(SeesPIDTest, ZeroGainIsStock)
{
	Random random;
	AttitudeControl off;
	off.setProportionalGain(Vector3f(6.5f, 6.5f, 2.8f), .4f);
	off.setRateLimit(Vector3f(10.f, 10.f, 10.f));
	off.setFeedForward(0.f, kCutoff);

	for (int i = 0; i < 1000; i++) {
		AttitudeControl stock;
		stock.setProportionalGain(Vector3f(6.5f, 6.5f, 2.8f), .4f);
		stock.setRateLimit(Vector3f(10.f, 10.f, 10.f));

		const Quatf q = random.attitude();
		const Quatf target = q * Quatf(AxisAnglef(random.vector(0.2f)));
		const float dt = random.dt();
		const Vector3f actual = step(off, target, q, dt);
		const Vector3f expected = step(stock, target, q, dt);

		for (int axis = 0; axis < 3; axis++) {
			EXPECT_EQ(actual(axis), expected(axis));
		}
	}
}

// The feed-forward is added before the rate limit, so the rate setpoint never exceeds it
TEST(SeesPIDTest, RateLimitHolds)
{
	Random random;

	for (int n = 0; n < kCases; n++) {
		const float limit = random.uniform(0.5f, 5.f);
		AttitudeControl ac = feedForwardOnly(random.uniform(0.f, 1.f), limit);
		const Quatf q0 = random.attitude();
		const Vector3f r = random.vector(50.f);
		const float dt = random.dt();

		for (int i = 0; i < 100; i++) {
			const Vector3f ff = step(ac, q0 * Quatf(AxisAnglef(r * (i * dt))), Quatf(), dt);

			for (int axis = 0; axis < 3; axis++) {
				EXPECT_LE(fabsf(ff(axis)), limit);
			}
		}
	}
}
