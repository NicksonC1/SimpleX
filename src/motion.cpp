#include "motion.hpp"

#include <algorithm>
#include <cmath>

constexpr double kPi = 3.14159265358979323846;

double poseX = 0.0;
double poseY = 0.0;

double wheelTravelPerInch() {
	return 360.0 / (kPi * wheelDiameter);
}

double headingError(double target, double current) {
	double error = target - current;
	while (error > 180.0) error -= 360.0;
	while (error < -180.0) error += 360.0;
	return error;
}

void stopDrive() {
	frontLeft.brake();
	frontRight.brake();
	backLeft.brake();
	backRight.brake();
}

double averageDrivePosition() {
	return (std::abs(frontLeft.get_position()) + std::abs(frontRight.get_position()) +
	        std::abs(backLeft.get_position()) + std::abs(backRight.get_position())) /
	       4.0;
}

void driveDistance(double inches, int maxVoltage) {
	frontLeft.tare_position();
	frontRight.tare_position();
	backLeft.tare_position();
	backRight.tare_position();

	const double target = std::abs(inches) * wheelTravelPerInch();
	const int direction = inches >= 0.0 ? 1 : -1;

	while (target - averageDrivePosition() > driveTolerance * wheelTravelPerInch()) {
		const double error = target - averageDrivePosition();
		const int voltage = direction * std::clamp(static_cast<int>(error * driveKp), 1200, maxVoltage);
		frontLeft.move_voltage(voltage);
		frontRight.move_voltage(voltage);
		backLeft.move_voltage(voltage);
		backRight.move_voltage(voltage);
		pros::delay(10);
	}

	stopDrive();

	const double headingRad = imu.get_heading() * kPi / 180.0;
	poseX += inches * std::sin(headingRad);
	poseY += inches * std::cos(headingRad);
}

void strafeDistance(double inches, int maxVoltage) {
	frontLeft.tare_position();
	frontRight.tare_position();
	backLeft.tare_position();
	backRight.tare_position();

	const double target = std::abs(inches) * wheelTravelPerInch();
	const int direction = inches >= 0.0 ? 1 : -1;

	while (target - averageDrivePosition() > driveTolerance * wheelTravelPerInch()) {
		const double error = target - averageDrivePosition();
		const int voltage = direction * std::clamp(static_cast<int>(error * driveKp), 1200, maxVoltage);
		frontLeft.move_voltage(voltage);
		frontRight.move_voltage(-voltage);
		backLeft.move_voltage(-voltage);
		backRight.move_voltage(voltage);
		pros::delay(10);
	}

	stopDrive();

	const double headingRad = imu.get_heading() * kPi / 180.0;
	poseX += inches * std::cos(headingRad);
	poseY -= inches * std::sin(headingRad);
}

void turnToHeading(double heading, int maxVoltage) {
	double error = headingError(heading, imu.get_heading());
	while (std::abs(error) > turnTolerance) {
		const int voltage = std::clamp(static_cast<int>(error * turnKp), -maxVoltage, maxVoltage);
		frontLeft.move_voltage(voltage);
		frontRight.move_voltage(-voltage);
		backLeft.move_voltage(voltage);
		backRight.move_voltage(-voltage);
		pros::delay(10);
		error = headingError(heading, imu.get_heading());
	}

	stopDrive();
}

void turnToPoint(double x, double y, int maxVoltage) {
	const double heading = std::atan2(x - poseX, y - poseY) * 180.0 / kPi;
	turnToHeading(heading < 0.0 ? heading + 360.0 : heading, maxVoltage);
}

void moveToPose(double x, double y, double theta, int maxVoltage) {
	double dx = x - poseX;
	double dy = y - poseY;
	double distance = std::hypot(dx, dy);

	frontLeft.tare_position();
	frontRight.tare_position();
	backLeft.tare_position();
	backRight.tare_position();
	double lastFl = 0.0;
	double lastFr = 0.0;
	double lastBl = 0.0;
	double lastBr = 0.0;

	while (distance > driveTolerance || std::abs(headingError(theta, imu.get_heading())) > turnTolerance) {
		const double headingRad = imu.get_heading() * kPi / 180.0;
		const double forward = dx * std::sin(headingRad) + dy * std::cos(headingRad);
		const double strafe = dx * std::cos(headingRad) - dy * std::sin(headingRad);
		const double turn = headingError(theta, imu.get_heading()) * moveTurnKp / moveKp;

		double fl = (forward + strafe) * moveKp + turn * moveKp;
		double fr = (forward - strafe) * moveKp - turn * moveKp;
		double bl = (forward - strafe) * moveKp + turn * moveKp;
		double br = (forward + strafe) * moveKp - turn * moveKp;
		const double scale = std::max({static_cast<double>(maxVoltage), std::abs(fl), std::abs(fr), std::abs(bl), std::abs(br)});

		frontLeft.move_voltage(static_cast<int>(fl / scale * maxVoltage));
		frontRight.move_voltage(static_cast<int>(fr / scale * maxVoltage));
		backLeft.move_voltage(static_cast<int>(bl / scale * maxVoltage));
		backRight.move_voltage(static_cast<int>(br / scale * maxVoltage));

		pros::delay(10);

		const double newFl = frontLeft.get_position();
		const double newFr = frontRight.get_position();
		const double newBl = backLeft.get_position();
		const double newBr = backRight.get_position();
		const double dfl = newFl - lastFl;
		const double dfr = newFr - lastFr;
		const double dbl = newBl - lastBl;
		const double dbr = newBr - lastBr;
		lastFl = newFl;
		lastFr = newFr;
		lastBl = newBl;
		lastBr = newBr;

		const double localForward = (dfl + dfr + dbl + dbr) / 4.0 / wheelTravelPerInch();
		const double localStrafe = (dfl - dfr - dbl + dbr) / 4.0 / wheelTravelPerInch();
		const double newHeadingRad = imu.get_heading() * kPi / 180.0;
		poseX += localStrafe * std::cos(newHeadingRad) + localForward * std::sin(newHeadingRad);
		poseY += localForward * std::cos(newHeadingRad) - localStrafe * std::sin(newHeadingRad);

		dx = x - poseX;
		dy = y - poseY;
		distance = std::hypot(dx, dy);
	}

	poseX = x;
	poseY = y;
	turnToHeading(theta);
}
