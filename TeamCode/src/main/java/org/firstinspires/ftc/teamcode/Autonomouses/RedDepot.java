package org.firstinspires.ftc.teamcode.Autonomouses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Baguette;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.util.MecanumDriveTranslationalPID;
import org.firstinspires.ftc.teamcode.util.MecanumDriveTurnPID;
import org.firstinspires.ftc.teamcode.util.Pose2D;
import org.firstinspires.ftc.teamcode.vision.camera;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.Wait.waitTime;

@Autonomous(group = "Comp", name = "red depot")
public class RedDepot extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        /*Baguette.initializeBaguette(this);
        MecanumDriveTranslationalPID move = new MecanumDriveTranslationalPID();
        MecanumDriveTurnPID turn = new MecanumDriveTurnPID();

        Baguette.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Baguette.webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                //This will be called if the camera could not be opened
            }
        });

        waitForStart();
        if(opModeIsActive()) {
            camera.MarkerPosition place = Baguette.pipeline.getAnalysis();

            move.tPID(new Pose2D(0, 4, 0));
            waitTime(500);
            move.tPID(new Pose2D(-24, 0, 0));
            waitTime(500);
            move.tPID(new Pose2D(0, 12, 0));
            waitTime(500);

            if (place == camera.MarkerPosition.LEFT) {
                Baguette.arm.dropArm(1);
            }
            else if (place == camera.MarkerPosition.CENTER) {
                Baguette.arm.dropArm(2);
            }
            else if (place == camera.MarkerPosition.RIGHT) {
                Baguette.arm.dropArm(3);
            }
            else {
                Baguette.arm.dropArm(3);
                telemetry.addData("get analysis", " didn't work in auto");
            }

            move.tPID(new Pose2D(0, -4, 0));
            waitTime(500);
            turn.turnPID(3 * Math.PI/2);
            waitTime(500);
            move.tPID(new Pose2D(0, 24, 0));
            waitTime(500);
        }*/
    }
}
