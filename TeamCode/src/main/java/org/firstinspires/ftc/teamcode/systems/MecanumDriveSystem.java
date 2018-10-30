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

/**
 * Created by Mahim on 12/4/2017.
 */

public class MecanumDriveSystem extends Mechanism {
    private DcMotor frontLeftMotor;
    private DcMotor rearLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor rearRightMotor;
    private BNO055IMU imu;

    public MecanumDriveSystem(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
    }

    public MecanumDriveSystem() {}

    @Override
    public void init(HardwareMap hwMap) {
        this.frontLeftMotor = hwMap.get(DcMotor.class, "front left motor");
        this.rearLeftMotor = hwMap.get(DcMotor.class, "rear left motor");
        this.frontRightMotor = hwMap.get(DcMotor.class, "front right motor");
        this.rearRightMotor = hwMap.get(DcMotor.class, "rear right motor");
        this.rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm
                = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public enum Direction {
        FORWARD, REVERSE
    }

    public void drive(double x, double y, double turn) {
        this.frontLeftMotor.setPower(y - x - turn);
        this.rearLeftMotor.setPower(y + x - turn);
        this.frontRightMotor.setPower(y + x + turn);
        this.rearRightMotor.setPower(y - x + turn);
    }

    public void drive(double left, double right) {
        this.frontLeftMotor.setPower(left);
        this.rearLeftMotor.setPower(left);
        this.frontRightMotor.setPower(right);
        this.rearRightMotor.setPower(right);
    }

    public void stop() {
        this.frontLeftMotor.setPower(0.0);
        this.rearLeftMotor.setPower(0.0);
        this.frontRightMotor.setPower(0.0);
        this.rearRightMotor.setPower(0.0);
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

        while (linearOpMode.opModeIsActive() && Math.abs(
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
