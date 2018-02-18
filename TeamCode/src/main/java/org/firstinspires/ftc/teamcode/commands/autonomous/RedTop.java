package org.firstinspires.ftc.teamcode.commands.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.ColorSensorSystem;
import org.firstinspires.ftc.teamcode.systems.tools.Direction;
import org.firstinspires.ftc.teamcode.systems.MecanumDriveSystem;

/**
 * Created by Mahim on 1/12/2018.
 */
@Autonomous(name = "red alliance top", group = "red alliance")
public class RedTop extends LinearOpMode {
    private MecanumDriveSystem  mecanumDriveSystem;
    private ColorSensorSystem   colorSensorSystem;
    private ElapsedTime         runtime = new ElapsedTime();

    private void initialize() {
        this.mecanumDriveSystem = new MecanumDriveSystem(hardwareMap);
        this.colorSensorSystem  = new ColorSensorSystem(hardwareMap);
        this.colorSensorSystem.setInitPosition();
    }

    private void knockDownBlueJewel() {
        this.colorSensorSystem.goDown();
        sleep(1500);
        if(colorSensorSystem.isBlue()) {
            mecanumDriveSystem.drive(0.5, 0.5, Direction.REVERSE);
            sleep(250);
        } else if(colorSensorSystem.isRed()) {
            mecanumDriveSystem.drive(0.5, 0.5, Direction.FORWARD);
            sleep(250);
        } else {
            mecanumDriveSystem.stop();
        }
        this.colorSensorSystem.setInitPosition();
        sleep(1000);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        runtime.reset();
        int count = 0;

        while (opModeIsActive() && (count < 1)) {
            knockDownBlueJewel();
            mecanumDriveSystem.stop();
            sleep(1000);
            this.mecanumDriveSystem.drive(0.3, 0.5, Direction.FORWARD);
            sleep(1000);
            mecanumDriveSystem.stop();
            count++;
        }
    }
}