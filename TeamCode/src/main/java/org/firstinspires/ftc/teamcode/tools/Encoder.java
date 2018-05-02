package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.tools.PID.PIDSource;

/**
 * Created by Mahim on 5/2/18.
 */

public class Encoder implements PIDSource {
    private DcMotor motor;

    public Encoder(DcMotor motor) {
        this.motor = motor;
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.Displacement;
    }

    @Override
    public double PIDGet() {
        return motor.getCurrentPosition();
    }
}
