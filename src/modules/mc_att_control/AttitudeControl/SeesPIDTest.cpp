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
 * docs/sees/26_proposed_controller.md. Each test checks one property over random cases.
 */

#include <gtest/gtest.h>
#include <AttitudeControl.hpp>
#include <random>
#include <vector>

using namespace matrix;

static constexpr float kDt = 0.004f; // 250 Hz attitude loop
static constexpr int kOneSecond = 250; // steps
static constexpr int kCases = 20;
static constexpr float kGain = 0.138f;
static constexpr float kCutoff = 20.f;
static constexpr float kOmegaF = M_TWOPI_F * kCutoff;
static constexpr float kTol = 1e-4f;

static std::mt19937 rng(26);

static float uniform(float lo, float hi) { return std::uniform_real_distribution<float>(lo, hi)(rng); }
static Vector3f randomVector(float max) { return Vector3f(uniform(-max, max), uniform(-max, max), uniform(-max, max)); }

static Quatf randomAttitude()
{
	std::normal_distribution<float> n;
	Quatf q(n(rng), n(rng), n(rng), n(rng));
	q.normalize();
	return q;
}

static Vector3f rollPitch(const Vector3f &v) { return Vector3f(v(0), v(1), 0.f); }

// Attitude control with only the feed-forward, so update() returns the feed-forward
static AttitudeControl feedForwardOnly(float gain = kGain, const Vector3f &rate_limit = Vector3f(100.f, 100.f, 100.f))
{
	AttitudeControl ac;
	ac.setProportionalGain(Vector3f(), 1.f);
	ac.setRateLimit(rate_limit);
	ac.setFeedForward(gain, kCutoff);
	return ac;
}

// One attitude loop step with the drone at attitude q
static Vector3f tick(AttitudeControl &ac, const Quatf &q)
{
	ac.updateFeedForward(q, kDt);
	return ac.update(q);
}

static Vector3f step(AttitudeControl &ac, const Quatf &target, const Quatf &q)
{
	ac.setAttitudeSetpoint(target, 0.f);
	return tick(ac, q);
}

// The target at time t, starting at q0 and turning at rate r in its own frame
static Quatf turning(const Quatf &q0, const Vector3f &r, float t) { return q0 * Quatf(AxisAnglef(r * t)); }

// Feed-forward over one second of a turning target, with the drone at a fixed offset from the target
static std::vector<Vector3f> turn(const Quatf &q0, const Vector3f &r, const Quatf &offset = Quatf())
{
	AttitudeControl ac = feedForwardOnly();
	std::vector<Vector3f> ff;

	for (int i = 0; i < kOneSecond; i++) {
		const Quatf target = turning(q0, r, i * kDt);
		ff.push_back(step(ac, target, target * offset));
	}

	return ff;
}

// (5) only has s * theta_T, so a constant target gives zero, whatever the drone's attitude
TEST(SeesPIDTest, ConstantTargetGivesZero)
{
	for (int n = 0; n < kCases; n++) {
		AttitudeControl ac = feedForwardOnly();
		const Quatf target = randomAttitude();

		for (int i = 0; i < kOneSecond; i++) {
			EXPECT_LT(step(ac, target, randomAttitude()).norm(), kTol);
		}
	}
}

// F has unit DC gain, so a target turning at r settles to K_ff * r, on roll and pitch only
TEST(SeesPIDTest, SteadyRateGivesGainTimesRate)
{
	for (int n = 0; n < kCases; n++) {
		const Vector3f r = randomVector(1.f);
		EXPECT_LT((turn(randomAttitude(), r).back() - kGain * rollPitch(r)).norm(), kTol);
	}
}

// The rate setpoint is in the body frame, so a drone offset from the target sees the target's rate rotated
TEST(SeesPIDTest, RateIsInBodyFrame)
{
	for (int n = 0; n < kCases; n++) {
		const Vector3f r = randomVector(1.f);
		const Quatf offset = randomAttitude();
		const Vector3f r_body = offset.inversed().rotateVector(r);
		EXPECT_LT((turn(randomAttitude(), r, offset).back() - kGain * rollPitch(r_body)).norm(), kTol);
	}
}

// (5) is linear in theta_T, so at every step -r negates the feed-forward and 2r doubles it
TEST(SeesPIDTest, LinearInRate)
{
	for (int n = 0; n < kCases; n++) {
		const Quatf q0 = randomAttitude();
		const Vector3f r = randomVector(1.f);
		const std::vector<Vector3f> ff = turn(q0, r), ff_neg = turn(q0, -r), ff_double = turn(q0, 2.f * r);

		for (int i = 0; i < kOneSecond; i++) {
			EXPECT_LT((ff_neg[i] + ff[i]).norm(), kTol);
			EXPECT_LT((ff_double[i] - 2.f * ff[i]).norm(), kTol);
		}
	}
}

// q and -q are the same attitude, so flipping the setpoint quaternion's sign changes nothing
TEST(SeesPIDTest, QuaternionSignDoesNotMatter)
{
	for (int n = 0; n < kCases; n++) {
		const Quatf q0 = randomAttitude();
		const Vector3f r = randomVector(1.f);
		const std::vector<Vector3f> ff = turn(q0, r);
		AttitudeControl ac = feedForwardOnly();

		for (int i = 0; i < kOneSecond; i++) {
			const Quatf target = turning(q0, r, i * kDt);
			const Quatf flipped = (rng() % 2) ? Quatf(-target(0), -target(1), -target(2), -target(3)) : target;
			EXPECT_LT((step(ac, flipped, target) - ff[i]).norm(), kTol);
		}
	}
}

// A heading reset moves the setpoint without the target moving, so it gives zero
TEST(SeesPIDTest, HeadingResetGivesZero)
{
	for (int n = 0; n < kCases; n++) {
		AttitudeControl ac = feedForwardOnly();
		const Quatf q = randomAttitude();
		step(ac, randomAttitude(), q);
		ac.adaptAttitudeSetpoint(Quatf(AxisAnglef(0.f, 0.f, uniform(-M_PI_F, M_PI_F))));
		EXPECT_LT(tick(ac, q).norm(), kTol);
	}
}

// F preserves area, so after a change delta in the target the feed-forward integrates to K_ff * delta,
// however the change is spread out and however often the setpoint arrives
TEST(SeesPIDTest, StepIntegralIsGainTimesStep)
{
	for (int n = 0; n < kCases; n++) {
		AttitudeControl ac = feedForwardOnly();
		const Vector3f delta = randomVector(0.3f);
		const int ramp_steps = 1 + rng() % 50;
		const int setpoint_every = 1 + rng() % 10;
		step(ac, Quatf(), Quatf());
		Quatf target;
		Vector3f integral;

		for (int i = 0; i < kOneSecond; i++) {
			if (i % setpoint_every == 0) {
				target = Quatf(AxisAnglef(delta * fminf((i + 1.f) / ramp_steps, 1.f)));
			}

			integral += step(ac, target, Quatf()) * kDt;
		}

		EXPECT_LT((integral - kGain * rollPitch(delta)).norm(), kTol);
	}
}

// F is first order, so a sudden step delta peaks at no more than K_ff * omega_f * delta
TEST(SeesPIDTest, StepPeakIsBounded)
{
	for (int n = 0; n < kCases; n++) {
		AttitudeControl ac = feedForwardOnly();
		const Vector3f delta = randomVector(0.3f);
		step(ac, Quatf(), Quatf());

		for (int i = 0; i < kOneSecond; i++) {
			const Vector3f ff = step(ac, Quatf(AxisAnglef(delta)), Quatf());

			for (int axis = 0; axis < 2; axis++) {
				EXPECT_LE(fabsf(ff(axis)), kGain * kOmegaF * fabsf(delta(axis)) + kTol);
			}
		}
	}
}

// With K_ff = 0 the rate setpoint is exactly stock PX4, for any inputs
TEST(SeesPIDTest, ZeroGainIsStock)
{
	AttitudeControl stock, off;

	for (AttitudeControl *ac : {&stock, &off}) {
		ac->setProportionalGain(Vector3f(6.5f, 6.5f, 2.8f), .4f);
		ac->setRateLimit(Vector3f(3.8f, 3.8f, 3.5f));
	}

	off.setFeedForward(0.f, kCutoff);

	for (int i = 0; i < kCases * kOneSecond; i++) {
		const Quatf target = randomAttitude();
		const Quatf q = randomAttitude();
		stock.setAttitudeSetpoint(target, 0.f);
		EXPECT_EQ(step(off, target, q), stock.update(q));
	}
}

// The feed-forward is added before the rate limit, so the rate setpoint never exceeds it
TEST(SeesPIDTest, RateLimitHolds)
{
	for (int n = 0; n < kCases; n++) {
		const Vector3f limit(uniform(0.5f, 5.f), uniform(0.5f, 5.f), uniform(0.5f, 5.f));
		AttitudeControl ac = feedForwardOnly(uniform(0.f, 1.f), limit);
		const Quatf q0 = randomAttitude();
		const Vector3f r = randomVector(50.f);

		for (int i = 0; i < kOneSecond; i++) {
			const Vector3f ff = step(ac, turning(q0, r, i * kDt), Quatf());

			for (int axis = 0; axis < 3; axis++) {
				EXPECT_LE(fabsf(ff(axis)), limit(axis));
			}
		}
	}
}
