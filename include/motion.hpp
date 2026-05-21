#ifndef MOTION_HPP
#define MOTION_HPP

#include "main.h"
#include "pid.hpp"

class CustomIMU : public pros::Imu {
  public:
	CustomIMU(std::uint8_t port, double scalar);

	double get_rotation() const override;
	double get_heading() const override;

  private:
	const double m_scalar;
};

extern CustomIMU imu;
extern pros::Motor frontLeft;
extern pros::Motor frontRight;
extern pros::Motor backLeft;
extern pros::Motor backRight;

extern const double wheelDiameter;
extern genesis::ControllerSettings lateralController;
extern genesis::ControllerSettings angularController;
extern const int defaultDrivePower;
extern const int defaultTurnPower;

void driveDistance(double inches, int maxPower = defaultDrivePower);
void strafeDistance(double inches, int maxPower = defaultDrivePower);
void turnToHeading(double heading, int maxPower = defaultTurnPower);
void turnToPoint(double x, double y, int maxPower = defaultTurnPower);
void moveToPose(double x, double y, double theta, int maxPower = defaultDrivePower);
void arcadeDrive(int forward, int strafe, int turn);

#endif
