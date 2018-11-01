package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Mahim on 1/9/18.
 */

public class ArmSystem extends Mechanism {
    private Servo leftArmServo;
    private Servo rightArmServo;
    private DcMotor armMotor;

    public ArmSystem(LinearOpMode opMode) {
        this.linearOpMode = opMode;
    }

    public ArmSystem() {}

    @Override
    public void init(HardwareMap hwMap) {
        this.leftArmServo = hwMap.get(Servo.class, "left arm servo");
        this.rightArmServo = hwMap.get(Servo.class, "right arm servo");
        this.armMotor = hwMap.get(DcMotor.class, "arm motor");
        this.rightArmServo.setDirection(Servo.Direction.REVERSE);
        this.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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