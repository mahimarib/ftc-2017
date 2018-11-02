package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.RobotMap;

public class DriveSystem extends Mechanism {
    private DcMotor frontLeftMotor;
    private DcMotor rearLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor rearRightMotor;
    private BNO055IMU imu;

    public DriveSystem(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public DriveSystem() {}

    @Override
    public void init(HardwareMap hwMap) {
        frontLeftMotor = hwMap.dcMotor.get(RobotMap.FRONT_LEFT_MOTOR);
        rearLeftMotor = hwMap.dcMotor.get(RobotMap.REAR_LEFT_MOTOR);
        frontRightMotor = hwMap.dcMotor.get(RobotMap.FRONT_RIGHT_MOTOR);
        rearRightMotor = hwMap.dcMotor.get(RobotMap.REAR_RIGHT_MOTOR);

        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm
                = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, RobotMap.GYRO);
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public enum Direction {
        FORWARD, REVERSE
    }

    public void drive(double x, double y, double turn) {
        frontLeftMotor.setPower(y - x - turn);
        rearLeftMotor.setPower(y + x - turn);
        frontRightMotor.setPower(y + x + turn);
        rearRightMotor.setPower(y - x + turn);
    }

    public void drive(double left, double right) {
        frontLeftMotor.setPower(left);
        rearLeftMotor.setPower(left);
        frontRightMotor.setPower(right);
        rearRightMotor.setPower(right);
    }

    public void stop() {
        frontLeftMotor.setPower(0.0);
        rearLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        rearRightMotor.setPower(0.0);
    }

    public void drive(
            double leftSpeed, double rightSpeed, Direction direction) {
        switch (direction) {
            case FORWARD:
                drive(-leftSpeed, -rightSpeed);
                break;
            case REVERSE:
                drive(leftSpeed, rightSpeed);
                break;
        }
    }

    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Turns to the desired given angle using PIDs
     * turn left is +angle, turn right is -angle
     *
     * @param targetAngle target angle in degrees
     */
    public void turn(double targetAngle) {
        targetAngle %= 360;
        targetAngle += getAngle();

        double integral = 0;
        double prevError = 0;

        double kP = 0.03;
        double kI = 0;
        double kD = 0.018;

        double maxSpeed = 1.0;

        while (opMode.opModeIsActive() && Math.abs(
                getAngleError(targetAngle)) > 1.5) {
            double error = getAngleError(targetAngle);

            integral += error;
            double derivative = error - prevError;

            double kP_output = kP * error;
            double kI_output = kI * integral;
            double kD_output = kD * derivative;

            double output = Range.clip(
                    kP_output + kI_output - kD_output, -maxSpeed, maxSpeed);

            prevError = error;

            drive(-output, output);

            opMode.telemetry.addData(
                    "Heading: ", "%.2f : %.2f", targetAngle, getAngle());
            opMode.telemetry.addData("Velocity: ", "%.2f", output);
            getSpeed(opMode.telemetry);
            opMode.telemetry.update();
        }
        stop();
    }

    private double getAngleError(double targetAngle) {
        return targetAngle - getAngle();
    }

    public void getSpeed(Telemetry telemetry) {
        telemetry.addData("front left", frontLeftMotor.getPower());
        telemetry.addData("rear left", rearLeftMotor.getPower());
        telemetry.addData("front right", frontRightMotor.getPower());
        telemetry.addData("rear right", rearRightMotor.getPower());
    }
}
