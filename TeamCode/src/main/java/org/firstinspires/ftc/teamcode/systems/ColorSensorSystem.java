package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotMap;

public class ColorSensorSystem extends Mechanism {
    private ColorSensor colorSensor;
    private Servo servo;
    private static final double INIT_POSITION = 0.0;
    private static final double DOWN_POSITION = 1.0;

    public ColorSensorSystem(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public ColorSensorSystem() {}

    @Override
    public void init(HardwareMap hwMap) {
        colorSensor = hwMap.colorSensor.get(RobotMap.COLOR_SENSOR);
        servo = hwMap.servo.get(RobotMap.COLOR_SENSOR_SERVO);
    }

    public void setInitPosition() {
        servo.setPosition(INIT_POSITION);
    }

    public void goDown() {
        servo.setPosition(DOWN_POSITION);
    }

    private int getRed() {
        return colorSensor.red();
    }

    private int getBlue() {
        return colorSensor.blue();
    }

    private int getGreen() {
        return colorSensor.green();
    }

    public boolean isBlue() {
        return getBlue() > getRed() && getBlue() > getGreen();
    }

    public boolean isRed() {
        return getRed() > getBlue() && getRed() > getGreen();
    }
}