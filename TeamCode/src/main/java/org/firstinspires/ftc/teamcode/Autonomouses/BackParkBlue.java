package org.firstinspires.ftc.teamcode.Autonomouses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Baguette;
import org.firstinspires.ftc.teamcode.util.MecanumDriveTranslationalPID;
import org.firstinspires.ftc.teamcode.util.Pose2D;

@Autonomous(group = "Comp", name = "back park blue")
public class BackParkBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        /*Baguette.initializeBaguette(this);

        MecanumDriveTranslationalPID move = new MecanumDriveTranslationalPID();

        waitForStart();
        if (opModeIsActive()) {
            move.tPID(new Pose2D(24, 0, 0));
            wait(1000);

            move.tPID(new Pose2D(0, 24, 0));
            wait(1000);
        }*/
    }
}
