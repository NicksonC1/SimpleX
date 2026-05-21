#ifndef MOTION_HPP
#define MOTION_HPP

#include "main.h"

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
extern const double driveKp;
extern const double turnKp;
extern const double moveKp;
extern const double moveTurnKp;
extern const double driveTolerance;
extern const double turnTolerance;
extern const int defaultDriveVoltage;
extern const int defaultTurnVoltage;

void driveDistance(double inches, int maxVoltage = defaultDriveVoltage);
void strafeDistance(double inches, int maxVoltage = defaultDriveVoltage);
void turnToHeading(double heading, int maxVoltage = defaultTurnVoltage);
void turnToPoint(double x, double y, int maxVoltage = defaultTurnVoltage);
void moveToPose(double x, double y, double theta, int maxVoltage = defaultDriveVoltage);

#endif
