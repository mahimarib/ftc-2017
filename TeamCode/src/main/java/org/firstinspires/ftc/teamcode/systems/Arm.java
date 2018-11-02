package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotMap;

/**
 * The Arm is the class that is used to define all of the hardware that is
 * used for the robot's arm, the Arm must be instantiated then
 * initialized before it can be used.
 */
public class Arm extends Mechanism {
    /* Hardware members */
    private Servo leftArmServo;
    private Servo rightArmServo;
    private DcMotor armMotor;

    /* Constants */
    private static final float OPEN_POSITION = .45F;

    /**
     * The constructor used for autonomous. Sets the opMode to the current
     * LinearOpMode running.
     *
     * @param opMode LinearOpMode
     */
    public Arm(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    /**
     * Default constructor for the arm system.
     */
    public Arm() {}

    /**
     * Maps the hardware members to the robot's HardwareMap, as well as
     * initializing arm positions as well as configuring the state of the
     * hardware members before the robot is moving.
     *
     * @param hwMap robot's hardware map
     */
    @Override
    public void init(HardwareMap hwMap) {
        // mapping the hardware members
        leftArmServo = hwMap.servo.get(RobotMap.ARM_LEFT_SERVO);
        rightArmServo = hwMap.servo.get(RobotMap.ARM_RIGHT_SERVO);
        armMotor = hwMap.dcMotor.get(RobotMap.ARM_MOTOR);
        // reversing the right servo to be symmetric to the left servo
        rightArmServo.setDirection(Servo.Direction.REVERSE);
        // setting the range of the servos, since it opens too wide
        leftArmServo.scaleRange(0, OPEN_POSITION);
        // the right arm servo is set differently since reversing it makes
        // it's neutral position 1 and it moves to 0 when you increase it's
        // position
        rightArmServo.scaleRange(1 - OPEN_POSITION, 1);
    }

    /**
     * Moves the arm up.
     */
    public void goUp() {
        armMotor.setPower(1.0);
    }

    /**
     * Moves the arm down.
     */
    public void goDown() {
        armMotor.setPower(-1.0);
    }

    /**
     * Stops the arm in place.
     */
    public void stopArmMotor() {
        armMotor.setPower(0.0);
    }

    /**
     * This sets both of the arm servos to the same position. This method is
     * used in TeleOperated period.
     *
     * @param position the position to which the servos should move, a value
     *                 in the range [0.0, 1.0]
     */
    public void triggerArmServo(double position) {
        leftArmServo.setPosition(position);
        rightArmServo.setPosition(position);
    }

    /**
     * Used for logging by showing it on the phone screen by using telemetry.
     *
     * @return left arm servo position
     */
    public double getLeftServoPosition() {
        return leftArmServo.getPosition();
    }

    /**
     * Used for logging by showing it on the phone screen by using telemetry.
     *
     * @return right arm servo position.
     */
    public double getRightServoPosition() {
        return rightArmServo.getPosition();
    }
}