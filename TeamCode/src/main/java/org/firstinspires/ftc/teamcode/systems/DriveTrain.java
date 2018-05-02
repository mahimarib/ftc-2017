package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.tools.Encoder;
import org.firstinspires.ftc.teamcode.tools.Gyroscope;

/**
 * Created by Mahim on 4/27/18.
 */

public class DriveTrain {
    private DcMotor   frontLeftMotor;
    private DcMotor   rearLeftMotor;
    private DcMotor   frontRightMotor;
    private DcMotor   rearRightMotor;
    private Encoder   frontLeftEncoder;
    private Encoder   rearLeftEncoder;
    private Encoder   frontRightEncoder;
    private Encoder   rearRightEncoder;
    private Gyroscope gyroscope;

    public DriveTrain (HardwareMap hardwareMap) {
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "front left motor");
        rearLeftMotor   = hardwareMap.get(DcMotor.class, "rear left motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front right motor");
        rearRightMotor  = hardwareMap.get(DcMotor.class, "rear right motor");

        //TODO: set motors reverse

        frontLeftEncoder  = new Encoder(frontLeftMotor);
        rearLeftEncoder   = new Encoder(rearLeftMotor);
        frontRightEncoder = new Encoder(frontRightMotor);
        rearRightEncoder  = new Encoder(rearRightMotor);

        gyroscope = new Gyroscope(hardwareMap, "imu");
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

    public double getFrontRightSpeed() {
        return this.frontRightMotor.getPower();
    }

    public double getRearRightSpeed() {
        return this.rearRightMotor.getPower();
    }

    public double getFrontLeftSpeed() {
        return this.frontLeftMotor.getPower();
    }

    public double getRearLeftSpeed() {
        return  this.rearLeftMotor.getPower();
    }

    public int getFrontLeftEncoderTick() {
        return (int)(frontLeftEncoder.PIDGet());
    }

    public int getRearLeftEncoderTick() {
        return (int)(rearLeftEncoder.PIDGet());
    }

    public int getFrontRightEncoderTick() {
        return (int)(frontRightEncoder.PIDGet());
    }

    public int getRearRightEncoderTick() {
        return (int)(rearRightEncoder.PIDGet());
    }

    public double getAngle() {
        return gyroscope.PIDGet();
    }
}
