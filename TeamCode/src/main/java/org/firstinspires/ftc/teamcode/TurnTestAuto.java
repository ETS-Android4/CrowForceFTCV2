package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.MecanumDriveTranslationalPID;
import org.firstinspires.ftc.teamcode.util.MecanumDriveTurnPID;
import org.firstinspires.ftc.teamcode.util.Pose2D;

@Autonomous(name="turn test auto", group = "practice")
public class TurnTestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Baguette.initializeBaguette(this);

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {
            MecanumDriveTurnPID.turnPID(Math.PI/2);
        }
    }
}
