#include "main.h"
#include "motion.hpp"

pros::Motor frontLeft(1, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor frontRight(-2, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor backLeft(3, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor backRight(-4, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

CustomIMU::CustomIMU(std::uint8_t port, double scalar)
	: pros::Imu(port),
	  m_scalar(scalar) {}

double CustomIMU::get_rotation() const {
	return pros::c::imu_get_rotation(_port) * m_scalar;
}

double CustomIMU::get_heading() const {
	double heading = pros::c::imu_get_heading(_port) * m_scalar;
	while (heading >= 360.0) heading -= 360.0;
	while (heading < 0.0) heading += 360.0;
	return heading;
}

CustomIMU imu(1, 1.0); // checked
// pros::Imu imu(5);

const double wheelDiameter = 3.25;
const double driveKp = 900.0;
const double turnKp = 90.0;
const double moveKp = 900.0;
const double moveTurnKp = 90.0;
const double driveTolerance = 0.5;
const double turnTolerance = 1.0;
const int defaultDriveVoltage = 9000;
const int defaultTurnVoltage = 8000;

void initialize() {
	imu.reset(true);
}

void autonomous() {
	driveDistance(24);
	strafeDistance(12);
	turnToHeading(90);
	turnToPoint(48, 24);
	moveToPose(48, 24, 180);
}

void opcontrol() {}
