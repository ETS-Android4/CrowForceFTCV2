package org.firstinspires.ftc.teamcode.Autonomouses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Baguette;
import org.firstinspires.ftc.teamcode.util.MecanumDriveTranslationalPID;
import org.firstinspires.ftc.teamcode.util.Pose2D;

@Autonomous(group = "Comp", name = "forward park")
class ForwardPark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Baguette.initializeBaguette();

        MecanumDriveTranslationalPID move = new MecanumDriveTranslationalPID();

        waitForStart();
        if(opModeIsActive()) {
            move.tPID(new Pose2D(0, 24, 0));
        }

    }
}
