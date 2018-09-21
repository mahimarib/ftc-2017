package org.firstinspires.ftc.teamcode.hardware.systems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.hardware.systems.template.Mechanism;

/**
 * Created by Mahim on 4/27/18.
 */

public class DriveSystem extends Mechanism {
    private DcMotor   frontLeftMotor;
    private DcMotor   rearLeftMotor;
    private DcMotor   frontRightMotor;
    private DcMotor   rearRightMotor;
    private BNO055IMU imu;

    private static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public DriveSystem() {}

    public DriveSystem(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        frontLeftMotor  = hwMap.get(DcMotor.class, "front left motor");
        rearLeftMotor   = hwMap.get(DcMotor.class, "rear left motor");
        frontRightMotor = hwMap.get(DcMotor.class, "front right motor");
        rearRightMotor  = hwMap.get(DcMotor.class, "rear right motor");

        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public void initEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
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

        while (opMode.opModeIsActive() && Math.abs(getAngleError(targetAngle)) > 1.5) {
            double error = getAngleError(targetAngle);

            integral += error;
            double derivative = error - prevError;

            double kP_output = kP * error;
            double kI_output = kI * integral;
            double kD_output = kD * derivative;

            double output = Range.clip(kP_output + kI_output - kD_output, -maxSpeed, maxSpeed);

            prevError = error;

            drive(-output, output);

            opMode.telemetry.addData("Heading: ", "%.2f : %.2f", targetAngle, getAngle());
            opMode.telemetry.addData("Velocity: ", "%.2f", output);
            getSpeed(opMode.telemetry);
            opMode.telemetry.update();
        }
        stop();
    }

    /**
     * Drives to given distance
     *
     * @param leftDistanceInInches inches the left side drives
     * @param rightDistanceInInches inches the right side drives
     */
    public void driveDistance(double leftDistanceInInches, double rightDistanceInInches) {
        double leftTargetPos = (leftDistanceInInches * COUNTS_PER_INCH) + getLeftEncoderAvg();
        double rightTargetPos = (rightDistanceInInches * COUNTS_PER_INCH) + getRightEncoderAvg();

        double leftIntegral = 0;
        double leftPrevError = 0;

        double rightIntegral = 0;
        double rightPrevError = 0;

        double kP = 0.001;
        double kI = 0;
        double kD = 0.0001;

        double maxSpeed = 1.0;

        while (opMode.opModeIsActive() && (Math.abs(getLeftDistError(leftTargetPos)) > 200
                && Math.abs(getRightDistError(rightTargetPos)) > 200)) {
            double leftError = getLeftDistError(leftTargetPos);
            double rightError = getRightDistError(rightTargetPos);

            leftIntegral += leftError;
            rightIntegral += rightError;

            double leftDerivative = leftError - leftPrevError;
            double rightDerivative = rightError - rightPrevError;

            double leftP_output = kP * leftError;
            double leftI_output = kI * leftIntegral;
            double leftD_output = kD * leftDerivative;

            double rightP_output = kP * rightError;
            double rightI_output = kI * rightIntegral;
            double rightD_output = kD * rightDerivative;

            double leftOutput = Range.clip(leftP_output + leftI_output - leftD_output, -maxSpeed, maxSpeed);
            double rightOutput = Range.clip(rightP_output + rightI_output - rightD_output, -maxSpeed, maxSpeed);

            leftPrevError = leftError;
            rightPrevError = rightError;

            drive(leftOutput, rightOutput);

            getEncoderTicks(opMode.telemetry);
            getSpeed(opMode.telemetry);
            opMode.telemetry.update();
        }
        stop();
    }

    /**
     * drives to given distance
     *
     * @param distanceInInches distance in inches both sides drive
     */
    public void driveDistance(double distanceInInches) {
        driveDistance(distanceInInches, distanceInInches);
    }

    public double getLeftEncoderAvg() {
        return (double)(frontLeftMotor.getCurrentPosition() + rearLeftMotor.getCurrentPosition()) / 2;
    }

    public double getRightEncoderAvg() {
        return (double)(frontRightMotor.getCurrentPosition() + rearRightMotor.getCurrentPosition()) / 2;
    }

    // Get the difference in the target angle and the current heading
    private double getAngleError(double targetAngle) {
        return targetAngle - getAngle();
    }

    private double getLeftDistError(double targetPos) {
        return targetPos - getLeftEncoderAvg();
    }

    private double getRightDistError(double targetPos) {
        return  targetPos - getRightEncoderAvg();
    }

    public void getSpeed(Telemetry telemetry) {
        telemetry.addData("front left", frontLeftMotor.getPower());
        telemetry.addData("rear left", rearLeftMotor.getPower());
        telemetry.addData("front right", frontRightMotor.getPower());
        telemetry.addData("rear right", rearRightMotor.getPower());
    }

    public void getEncoderTicks(Telemetry telemetry) {
        telemetry.addData("front left encoder", frontLeftMotor.getCurrentPosition());
        telemetry.addData("rear left encoder", rearLeftMotor.getCurrentPosition());
        telemetry.addData("front right encoder", frontRightMotor.getCurrentPosition());
        telemetry.addData("rear right encoder", rearRightMotor.getCurrentPosition());
    }
}