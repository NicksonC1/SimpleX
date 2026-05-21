#include "main.h"
#include "motion.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
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

CustomIMU imu(5, 1.0); // checked

const double wheelDiameter = 3.25;
const int defaultDrivePower = 127;
const int defaultTurnPower = 127;

genesis::ControllerSettings lateralController(2,   // proportional gain (kP)
                                              0,   // integral gain (kI)
                                              0,   // derivative gain (kD)
                                              0,   // anti windup
                                              1,   // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3,   // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0    // maximum acceleration (slew)
);

genesis::ControllerSettings angularController(2,   // proportional gain (kP)
                                              0,   // integral gain (kI)
                                              0,   // derivative gain (kD)
                                              0,   // anti windup
                                              1,   // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3,   // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0    // maximum acceleration (slew)
);

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

void opcontrol() {

	while (1) {
		arcadeDrive(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
		            controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X),
		            controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

		pros::delay(10);
	}
}
