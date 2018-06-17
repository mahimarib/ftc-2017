package org.firstinspires.ftc.teamcode.hardware.systems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.hardware.systems.template.Mechanism;

/**
 * Created by Mahim on 4/27/18.
 */

public class DriveTrain extends Mechanism {
    private DcMotor   frontLeftMotor;
    private DcMotor   rearLeftMotor;
    private DcMotor   frontRightMotor;
    private DcMotor   rearRightMotor;
    private BNO055IMU imu;

    public DriveTrain() {}

    public DriveTrain(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        frontLeftMotor  = hwMap.get(DcMotor.class, "front left motor");
        rearLeftMotor   = hwMap.get(DcMotor.class, "rear left motor");
        frontRightMotor = hwMap.get(DcMotor.class, "front right motor");
        rearRightMotor  = hwMap.get(DcMotor.class, "rear right motor");

        this.rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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

    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    /**
     * Turn to a specified angle relative to the robot's starting position using an IMU.
     *
     * Robot will stop moving if any of three conditions occur:
     * <li>
     *  <ol>Move gets to the desired angle</ol>
     *  <ol>Move runs out of time</ol>
     *  <ol>Driver stops the running OpMode</ol>
     * </li>
     *
     * @param targetAngle   number of degrees to turn
     * @param timeoutS      amount of time before the move should stop
     */
    public void turn(double targetAngle, double timeoutS) {

        // Normalize input
        //targetAngle = targetAngle % 360;

        // Reset the timeout time
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        // Loop until a condition is met
        while (opMode.opModeIsActive() && Math.abs(getError(targetAngle)) > 1.5 && runtime.seconds() < timeoutS) {

            double velocity = getError(targetAngle) / 180 + 0.1; // this works
            //double velocity = Math.max(getError(targetAngle) / 180 * 2, 0.25); // to be tested why doesn't this work

            // Set motor power according to calculated angle to turn
            frontLeftMotor.setPower(velocity);
            frontRightMotor.setPower(-velocity);
            rearLeftMotor.setPower(velocity);
            rearRightMotor.setPower(-velocity);

            // Display heading for the driver
            opMode.telemetry.addData("Heading: ", "%.2f : %.2f", targetAngle, getHeading());
            opMode.telemetry.addData("Velocity: ", "%.2f", velocity);
            opMode.telemetry.update();
        }

        // Stop motor movement
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
    }

    /**
     * Get heading of the robot relative to the initialized position.
     * @return      heading of the robot
     */
    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    // Get the difference in the target angle and the current heading
    private double getError(double targetAngle) {
        double heading = getHeading();
        if (targetAngle > heading) {
            if (targetAngle - heading > 180) {
                return 360 - Math.abs(targetAngle) - Math.abs(heading);
            } else {
                return targetAngle - heading;
            }
        } else {
            if (targetAngle - heading > 180) {
                return -(360 - Math.abs(targetAngle) - Math.abs(heading));
            } else {
                return heading - targetAngle;
            }
        }
    }

    public void getSpeed(Telemetry t) {
        t.addData("front left", frontLeftMotor.getPower());
        t.addData("rear left", rearLeftMotor.getPower());
        t.addData("front right", frontRightMotor.getPower());
        t.addData("rear right", rearRightMotor.getPower());
    }
}
