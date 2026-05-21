#include "pid.hpp"
#include "util.hpp"

#include <cmath>

namespace genesis {
ControllerSettings::ControllerSettings(float proportionalGain,
                                       float integralGain,
                                       float derivativeGain,
                                       float antiWindupRange,
                                       float smallErrorRange,
                                       int smallErrorTimeout,
                                       float largeErrorRange,
                                       int largeErrorTimeout,
                                       float maxAcceleration)
	: kP(proportionalGain),
	  kI(integralGain),
	  kD(derivativeGain),
	  antiWindupRange(antiWindupRange),
	  smallErrorRange(smallErrorRange),
	  smallErrorTimeout(smallErrorTimeout),
	  largeErrorRange(largeErrorRange),
	  largeErrorTimeout(largeErrorTimeout),
	  maxAcceleration(maxAcceleration) {}

PID::PID(float kP, float kI, float kD, float windupRange, bool signFlipReset)
	: kP(kP),
	  kI(kI),
	  kD(kD),
	  windupRange(windupRange),
	  signFlipReset(signFlipReset) {}

float PID::update(float error) {
	integral += error;
	if (sgn(error) != sgn(prevError) && signFlipReset) integral = 0;
	if (std::fabs(error) > windupRange && windupRange != 0) integral = 0;

	const float derivative = error - prevError;
	prevError = error;

	return error * kP + integral * kI + derivative * kD;
}

void PID::reset() {
	integral = 0;
	prevError = 0;
}
} // namespace genesis
