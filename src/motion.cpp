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

int clampPower(double output, int maxPower) {
	const int power = std::clamp(static_cast<int>(output), -maxPower, maxPower);
	if (power == 0) return 0;
	if (std::abs(power) < 13) return power > 0 ? 13 : -13;
	return power;
}

void stopDrive() {
	frontLeft.brake();
	frontRight.brake();
	backLeft.brake();
	backRight.brake();
}

void arcadeDrive(int forward, int strafe, int turn) {
	const int deadband = 5;

	if (std::abs(forward) < deadband) forward = 0;
	if (std::abs(strafe) < deadband) strafe = 0;
	if (std::abs(turn) < deadband) turn = 0;

	const int fl = std::clamp(forward + strafe + turn, -127, 127);
	const int fr = std::clamp(forward - strafe - turn, -127, 127);
	const int bl = std::clamp(forward - strafe + turn, -127, 127);
	const int br = std::clamp(forward + strafe - turn, -127, 127);

	frontLeft.move(fl);
	frontRight.move(fr);
	backLeft.move(bl);
	backRight.move(br);
}

double averageDrivePosition() {
	return (std::abs(frontLeft.get_position()) + std::abs(frontRight.get_position()) +
	        std::abs(backLeft.get_position()) + std::abs(backRight.get_position())) /
	       4.0;
}

void driveDistance(double inches, int maxPower) {
	genesis::PID lateralPid(lateralController.kP, lateralController.kI, lateralController.kD, lateralController.antiWindupRange);
	lateralPid.reset();

	frontLeft.tare_position();
	frontRight.tare_position();
	backLeft.tare_position();
	backRight.tare_position();

	const double target = std::abs(inches) * wheelTravelPerInch();
	const int direction = inches >= 0.0 ? 1 : -1;

	while (target - averageDrivePosition() > lateralController.smallErrorRange * wheelTravelPerInch()) {
		const double error = target - averageDrivePosition();
		const int power = direction * clampPower(lateralPid.update(error), maxPower);
		frontLeft.move(power);
		frontRight.move(power);
		backLeft.move(power);
		backRight.move(power);
		pros::delay(10);
	}

	stopDrive();

	const double headingRad = imu.get_heading() * kPi / 180.0;
	poseX += inches * std::sin(headingRad);
	poseY += inches * std::cos(headingRad);
}

void strafeDistance(double inches, int maxPower) {
	genesis::PID lateralPid(lateralController.kP, lateralController.kI, lateralController.kD, lateralController.antiWindupRange);
	lateralPid.reset();

	frontLeft.tare_position();
	frontRight.tare_position();
	backLeft.tare_position();
	backRight.tare_position();

	const double target = std::abs(inches) * wheelTravelPerInch();
	const int direction = inches >= 0.0 ? 1 : -1;

	while (target - averageDrivePosition() > lateralController.smallErrorRange * wheelTravelPerInch()) {
		const double error = target - averageDrivePosition();
		const int power = direction * clampPower(lateralPid.update(error), maxPower);
		frontLeft.move(power);
		frontRight.move(-power);
		backLeft.move(-power);
		backRight.move(power);
		pros::delay(10);
	}

	stopDrive();

	const double headingRad = imu.get_heading() * kPi / 180.0;
	poseX += inches * std::cos(headingRad);
	poseY -= inches * std::sin(headingRad);
}

void turnToHeading(double heading, int maxPower) {
	genesis::PID angularPid(angularController.kP, angularController.kI, angularController.kD, angularController.antiWindupRange);
	angularPid.reset();

	double error = headingError(heading, imu.get_heading());
	while (std::abs(error) > angularController.smallErrorRange) {
		const int power = clampPower(angularPid.update(error), maxPower);
		frontLeft.move(power);
		frontRight.move(-power);
		backLeft.move(power);
		backRight.move(-power);
		pros::delay(10);
		error = headingError(heading, imu.get_heading());
	}

	stopDrive();
}

void turnToPoint(double x, double y, int maxPower) {
	const double heading = std::atan2(x - poseX, y - poseY) * 180.0 / kPi;
	turnToHeading(heading < 0.0 ? heading + 360.0 : heading, maxPower);
}

void moveToPose(double x, double y, double theta, int maxPower) {
	genesis::PID lateralPid(lateralController.kP, lateralController.kI, lateralController.kD, lateralController.antiWindupRange);
	genesis::PID angularPid(angularController.kP, angularController.kI, angularController.kD, angularController.antiWindupRange);
	lateralPid.reset();
	angularPid.reset();

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

	while (distance > lateralController.smallErrorRange ||
	       std::abs(headingError(theta, imu.get_heading())) > angularController.smallErrorRange) {
		const double headingRad = imu.get_heading() * kPi / 180.0;
		const double forward = dx * std::sin(headingRad) + dy * std::cos(headingRad);
		const double strafe = dx * std::cos(headingRad) - dy * std::sin(headingRad);
		const double lateralPower = lateralPid.update(distance);
		const double turn = angularPid.update(headingError(theta, imu.get_heading()));
		const double forwardPower = distance == 0.0 ? 0.0 : lateralPower * forward / distance;
		const double strafePower = distance == 0.0 ? 0.0 : lateralPower * strafe / distance;

		double fl = forwardPower + strafePower + turn;
		double fr = forwardPower - strafePower - turn;
		double bl = forwardPower - strafePower + turn;
		double br = forwardPower + strafePower - turn;
		const double scale = std::max({static_cast<double>(maxPower), std::abs(fl), std::abs(fr), std::abs(bl), std::abs(br)});

		frontLeft.move(static_cast<int>(fl / scale * maxPower));
		frontRight.move(static_cast<int>(fr / scale * maxPower));
		backLeft.move(static_cast<int>(bl / scale * maxPower));
		backRight.move(static_cast<int>(br / scale * maxPower));

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
