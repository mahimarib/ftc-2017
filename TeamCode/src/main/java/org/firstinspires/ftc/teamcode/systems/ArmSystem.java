package org.firstinspires.ftc.teamcode.systems;

import android.hardware.Sensor;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Mahim on 1/9/18.
 */

public class ArmSystem {
    private Servo   leftArmServo;
    private Servo   rightArmServo;
    private DcMotor armMotor;

    public ArmSystem(HardwareMap hardwareMap) {
        this.leftArmServo   = hardwareMap.get(Servo.class, "left arm servo");
        this.rightArmServo  = hardwareMap.get(Servo.class, "right arm servo");
        this.armMotor       = hardwareMap.get(DcMotor.class, "arm motor");
        this.leftArmServo.setDirection(Servo.Direction.REVERSE);
        this.rightArmServo.scaleRange(0.5, 1.0);
        this.leftArmServo.scaleRange(-1.0, 0.5);
    }

    public void goUp() {
        this.armMotor.setPower(1.0);
    }

    public void goDown() {
        this.armMotor.setPower(-1.0);
    }

    public void stopArmMotor() {
        this.armMotor.setPower(0.0);
    }

    public void triggerArmServo(double position) {
        this.leftArmServo.setPosition(position);
        this.rightArmServo.setPosition(position);
    }

    public double getLeftServoPosition() {
        return this.leftArmServo.getPosition();
    }

    public double getRightServoPosition() {
        return this.rightArmServo.getPosition();
    }
}
