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

import java.util.Arrays;

/**
 * Drivetrain class is the representation of the robot's drivetrain. The
 * Drivetrain must be instantiated and then initialized before it is used.
 * <p>
 * This Drivetrain is used for a mecanum drivetrain.
 */
public class Drivetrain extends Mechanism {
    /* Hardware members */
    private DcMotor frontLeftMotor;
    private DcMotor rearLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor rearRightMotor;
    private DcMotor[] motors;
    private BNO055IMU imu;

    /**
     * This constructor is used to instantiate the robot for autonomous, it
     * sets the opMode to the current linearOpMode that is running, it allows
     * it to use some functionalities that the opMode provides, like
     * {@link LinearOpMode#sleep(long)}.
     *
     * @param opMode LinearOpMode
     */
    public Drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    /**
     * The default constructor for the Drivetrain.
     */
    public Drivetrain() {}

    /**
     * Maps the hardware members to the robot's HardwareMap, as well as
     * initializing arm positions as well as configuring the state of the
     * hardware members before the robot is moving.
     *
     * @param hwMap robot's hardware map
     */
    @Override
    public void init(HardwareMap hwMap) {
        // mapping the drivetrain motors
        frontLeftMotor = hwMap.dcMotor.get(RobotMap.FRONT_LEFT_MOTOR);
        rearLeftMotor = hwMap.dcMotor.get(RobotMap.REAR_LEFT_MOTOR);
        frontRightMotor = hwMap.dcMotor.get(RobotMap.FRONT_RIGHT_MOTOR);
        rearRightMotor = hwMap.dcMotor.get(RobotMap.REAR_RIGHT_MOTOR);

        motors = new DcMotor[] {frontLeftMotor, rearLeftMotor,
                                frontRightMotor, rearRightMotor};
        // reversing the right side of the drivetrain so all the motors turn
        // the same direction
        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // setting it to brake mode so the motors uses force to stop itself
        // from turning freely when the power is set to zero.
        Arrays.stream(motors).forEach(x -> x.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE));
        // setting up the parameters for the built in gyroscope
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm
                = new JustLoggingAccelerationIntegrator();
        // mapping, and initializing the gyroscope
        imu = hwMap.get(BNO055IMU.class, RobotMap.GYRO);
        imu.initialize(parameters);
        // start logging the gyroscope, will the interval of 1 second
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    /**
     * Used to specify which direction the robot should move during autonomous
     */
    public enum Direction {
        FORWARD, REVERSE
    }

    /**
     * Used to set the drivetrain motors to drive as a mecanum drivetrain
     *
     * @param x    x component of the drive vector
     * @param y    y component of the drive vector
     * @param turn turn vector
     */
    public void drive(double x, double y, double turn) {
        frontLeftMotor.setPower(y - x - turn);
        rearLeftMotor.setPower(y + x - turn);
        frontRightMotor.setPower(y + x + turn);
        rearRightMotor.setPower(y - x + turn);
    }

    /**
     * Used to set the drivetrain motors to drive the robot in a
     * straightforward fashion
     *
     * @param left  speed for the left side of the drivetrain.
     * @param right speed for the right side of the drivetrain.
     */
    public void drive(double left, double right) {
        frontLeftMotor.setPower(left);
        rearLeftMotor.setPower(left);
        frontRightMotor.setPower(right);
        rearRightMotor.setPower(right);
    }

    /**
     * stops all of the motors of the drivetrain
     */
    public void stop() {
        Arrays.stream(motors).forEach(x -> x.setPower(0.0));
    }

    /**
     * Used to drive the robot in autonomous, because setting the motors to
     * -1.0 will actually drive the robot forward, this method will solve the
     * confusion.
     *
     * @param leftSpeed  speed for the left side of the drivetrain.
     * @param rightSpeed speed for the right side of the drivetrain.
     * @param direction  the direction the robot should move, forward or
     *                   reverse.
     */
    public void drive(
            double leftSpeed, double rightSpeed, Direction direction) {
        leftSpeed = Math.abs(leftSpeed);
        rightSpeed = Math.abs(rightSpeed);
        switch (direction) {
            case FORWARD:
                drive(-leftSpeed, -rightSpeed);
                break;
            case REVERSE:
                drive(leftSpeed, rightSpeed);
                break;
        }
    }

    /**
     * Gets the orientation of the robot, the z-axis or the yaw axis, the
     * orientation is always 0 degrees when the robot first starts.
     *
     * @return the orientation of the robot in degrees.
     */
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
        // scaling down the angle
        targetAngle %= 360;
        // setting the target angle based on the current orientation
        targetAngle += getAngle();

        double integral = 0;
        double prevError = 0;

        // PID coefficients
        double kP = 0.03;
        double kI = 0;
        double kD = 0.018;

        // setting the maximum speed the robot is allowed to go
        double maxSpeed = 1.0;

        while (opMode.opModeIsActive() && Math.abs(
                getAngleError(targetAngle)) > 1.5) {
            // calculation the error
            double error = getAngleError(targetAngle);
            // adding up the error
            integral += error;
            // calculating the change in the angle
            double derivative = error - prevError;
            // scaling the output with the coefficients
            double kP_output = kP * error;
            double kI_output = kI * integral;
            double kD_output = kD * derivative;
            // making sure the output is not above the maxSpeed
            double output = Range.clip(
                    kP_output + kI_output - kD_output, -maxSpeed, maxSpeed);
            // setting the previous error
            prevError = error;
            // actually using the output the drive the robot
            drive(-output, output);
            // printing out the values to test the PIDs
            opMode.telemetry.addData(
                    "Heading: ", "%.2f : %.2f", targetAngle, getAngle());
            opMode.telemetry.addData("Velocity: ", "%.2f", output);
            getSpeed(opMode.telemetry);
            opMode.telemetry.update();
        }
        // stopping the robot when it reaches the target
        stop();
    }

    /**
     * Calculates the error or how far the robot is from it's target angle.
     *
     * @param targetAngle the desired angle the robot should be in.
     * @return how far it is from the target angle.
     */
    private double getAngleError(double targetAngle) {
        return targetAngle - getAngle();
    }

    /**
     * prints out the speed of the motors, used to check if motor needs to be
     * reversed, or just checking the speed.
     *
     * @param telemetry the telemetry in the current opMode
     */
    public void getSpeed(Telemetry telemetry) {
        telemetry.addData("front left", frontLeftMotor.getPower());
        telemetry.addData("rear left", rearLeftMotor.getPower());
        telemetry.addData("front right", frontRightMotor.getPower());
        telemetry.addData("rear right", rearRightMotor.getPower());
    }
}
