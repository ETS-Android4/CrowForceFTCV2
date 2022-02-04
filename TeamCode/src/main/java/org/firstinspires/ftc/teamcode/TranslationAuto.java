package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.MecanumDriveTranslationalPID;
import org.firstinspires.ftc.teamcode.util.Pose2D;

@Autonomous(name="translation test auto", group = "practice")
public class TranslationAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Baguette.initializeBaguette(this);

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {
            MecanumDriveTranslationalPID.translatePID(new Pose2D(24, 0, 0));
        }
    }
}
