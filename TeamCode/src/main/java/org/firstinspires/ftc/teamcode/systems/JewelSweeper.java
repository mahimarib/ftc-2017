package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotMap;

/**
 * Jewel sweeper class is the representation of the robot's jewel sweeper.
 * This class must be instantiated before it can be used.
 */
public class JewelSweeper extends Mechanism {
    /* Hardware Members */
    private ColorSensor colorSensor;
    private Servo servo;

    /* Constants */
    private static final double INIT_POSITION = 0.0;
    private static final double DOWN_POSITION = 1.0;

    /**
     * The constructor used for autonomous. Sets the opMode to the current
     * LinearOpMode running.
     *
     * @param opMode LinearOpMode
     */
    public JewelSweeper(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    /**
     * Default constructor for the JewelSweeper.
     */
    public JewelSweeper() {}

    /**
     * Maps the hardware members to the robot's HardwareMap, as well as
     * initializing arm positions as well as configuring the state of the
     * hardware members before the robot is moving.
     *
     * @param hwMap robot's hardware map
     */
    @Override
    public void init(HardwareMap hwMap) {
        colorSensor = hwMap.colorSensor.get(RobotMap.COLOR_SENSOR);
        servo = hwMap.servo.get(RobotMap.COLOR_SENSOR_SERVO);
    }

    /**
     * Setting the jewel sweeper arm to its initial position so it does not
     * go past beyond the 18" height requirement.
     */
    public void setInitPosition() {
        servo.setPosition(INIT_POSITION);
    }

    public void goDown() {
        servo.setPosition(DOWN_POSITION);
    }

    /**
     * Get the Red values detected by the sensor as an int.
     *
     * @return reading, unscaled.
     */
    private int getRed() {
        return colorSensor.red();
    }

    /**
     * Get the Blue values detected by the sensor as an int.
     *
     * @return reading, unscaled.
     */
    private int getBlue() {
        return colorSensor.blue();
    }

    /**
     * Get the Red values detected by the sensor as an int.
     *
     * @return reading, unscaled.
     */
    private int getGreen() {
        return colorSensor.green();
    }

    /**
     * Compares the blue value to the red and green, and returns true if it
     * is greater.
     *
     * @return whether if it detects the blue jewel or not.
     */
    public boolean isBlue() {
        return getBlue() > getRed() && getBlue() > getGreen();
    }

    /**
     * Compares the red value to the blue and green value, and returns true
     * if it is greater.
     *
     * @return whether if it detects the red jewel or not.
     */
    public boolean isRed() {
        return getRed() > getBlue() && getRed() > getGreen();
    }
}