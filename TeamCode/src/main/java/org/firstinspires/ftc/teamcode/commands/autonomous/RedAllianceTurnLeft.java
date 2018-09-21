package org.firstinspires.ftc.teamcode.commands.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.systems.ArmSystem;
import org.firstinspires.ftc.teamcode.systems.DriveSystem;

/**
 * Created by Mahim on 12/9/2017.
 */
@Disabled
@Autonomous(name = "Red Alliance: Turn left", group = "Red Alliance")
public class RedAllianceTurnLeft extends LinearOpMode {
    private DriveSystem driveSystem;
    private ArmSystem armSystem;
    private ElapsedTime runtime = new ElapsedTime();

    private void initialize() {
        this.driveSystem = new DriveSystem(hardwareMap, gamepad1);
        this.armSystem = new ArmSystem(hardwareMap);
        this.armSystem.init();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        runtime.reset();
        int count = 0;

        while (opModeIsActive() && (count < 1)) {
            armSystem.setDownPosition();
            sleep(1000);
            knockDownBlueJewel();
            driveSystem.stop();
            sleep(1000);
            this.armSystem.setInitialPosition();
            sleep(2000);
            this.driveSystem.drive(-1.0, -0.9); // turn left
            sleep(2500);
            count++;
        }
    }

    private void knockDownBlueJewel() {
        this.armSystem.enableColorSensor();
        if(armSystem.isBlue()) {
            this.driveSystem.driveForward(1.0);
            sleep(500);
        } else if (armSystem.isRed()) {
            this.driveSystem.driveBackwards(1.0);
            sleep(500);
        }
    }
}