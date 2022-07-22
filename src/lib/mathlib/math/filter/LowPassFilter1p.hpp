/****************************************************************************
 *
 *   Copyright (C) 2012-2021 PX4 Development Team. All rights reserved.
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

/// @file	LowPassFilter1p.hpp
/// @brief	A class to implement a first order low pass filter
/// Based on simple 1st order RC model - https://en.wikipedia.org/wiki/Low-pass_filter

#pragma once

#include <mathlib/math/Functions.hpp>
#include <float.h>
#include <matrix/math.hpp>

namespace math
{

template<typename T>
class LowPassFilter1p
{
public:
	LowPassFilter1p() = default;

	LowPassFilter1p(float sample_freq, float cutoff_freq)
	{
		// set initial parameters
		set_cutoff_frequency(cutoff_freq);
		set_sample_frequency(sample_freq);
	}

	// Change filter parameters
	void set_cutoff_frequency(float cutoff_freq, bool reset_states = true)
	{
		_cutoff_freq = cutoff_freq;
		update_alpha(reset_states);
	}

	void set_sample_frequency(float sample_freq, bool reset_states = true)
	{
		_sample_freq = sample_freq;
		update_alpha(reset_states);
	}

	/**
	 * Add a new raw value to the filter
	 *
	 * @return retrieve the filtered result
	 */
	inline T apply(const T &sample)
	{
		_delay_element_1 += _alpha * (sample - _delay_element_1);

		return _delay_element_1;
	}

	/**
	 * Get the latest value
	 *
	 * @return retrieve the latest value
	 */
	inline T get()
	{
		return _delay_element_1;
	}

	// Filter array of samples in place
	inline void applyArray(T samples[], int num_samples)
	{
		for (int n = 0; n < num_samples; n++) {
			samples[n] = apply(samples[n]);
		}
	}

	// Return the cutoff frequency
	float get_cutoff_freq() const { return _cutoff_freq; }

	// Return the sample frequency
	float get_sample_freq() const { return _sample_freq; }

	// Reset the filter state to this value
	T reset(const T &sample)
	{
		_delay_element_1 = sample;
		return _delay_element_1;
	}

	void disable()
	{
		// no filtering
		_alpha = 1.f;
		_delay_element_1 = {};
	}

	bool disabled() {
		return fabsf(_alpha - 1.f) < 0.01f;
	}

protected:
	void update_alpha(bool reset_states) {
		if ((_sample_freq <= 0.f) || (_cutoff_freq <= 0.f) || (_cutoff_freq >= _sample_freq / 2)
		    || !isFinite(_sample_freq) || !isFinite(_cutoff_freq)) {
			disable();
			return;
		}

		float r = 2 * M_PI_F * _cutoff_freq / _sample_freq;
		_alpha = r / (1 + r);

		if (!isFinite(_alpha) || (_alpha < 0.f)) {
			disable();
			return;
		}

		// optionally reset delay elements on filter change
		if (reset_states) {
			_delay_element_1 = {};
		}
	}

protected:
	T _delay_element_1{}; // buffered output -1

	float _alpha{1.f}; // new sample gain

	float _cutoff_freq{0.f};
	float _sample_freq{0.f};
};

} // namespace math
