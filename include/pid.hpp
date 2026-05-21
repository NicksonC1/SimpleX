#ifndef PID_HPP
#define PID_HPP

namespace genesis {
struct ControllerSettings {
	ControllerSettings(float proportionalGain,
	                   float integralGain,
	                   float derivativeGain,
	                   float antiWindupRange,
	                   float smallErrorRange,
	                   int smallErrorTimeout,
	                   float largeErrorRange,
	                   int largeErrorTimeout,
	                   float maxAcceleration);

	float kP;
	float kI;
	float kD;
	float antiWindupRange;
	float smallErrorRange;
	int smallErrorTimeout;
	float largeErrorRange;
	int largeErrorTimeout;
	float maxAcceleration;
};

class PID {
  public:
	PID(float kP, float kI, float kD, float windupRange = 0, bool signFlipReset = true);

	float update(float error);
	void reset();

  private:
	float kP;
	float kI;
	float kD;
	float windupRange;
	bool signFlipReset;
	float integral = 0;
	float prevError = 0;
};
} // namespace genesis

#endif
