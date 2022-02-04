package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import static org.firstinspires.ftc.teamcode.vision.camera.MarkerPosition.CENTER;
import static org.firstinspires.ftc.teamcode.vision.camera.MarkerPosition.LEFT;
import static org.firstinspires.ftc.teamcode.vision.camera.MarkerPosition.RIGHT;

@Autonomous(group = "testing", name = "OpenCVTest")
class cameraTestAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        OpenCvWebcam webcam;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera pipeline;

        pipeline = new camera();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();
        if (opModeIsActive()) {
            if (pipeline.getAnalysis() == LEFT) {
                telemetry.addData("position: ",  "LEFT");
                telemetry.update();
            }
            else if (pipeline.getAnalysis() == RIGHT) {
                telemetry.addData("position: ",  "LEFT");
                telemetry.update();
            }
            else if (pipeline.getAnalysis() == CENTER) {
                telemetry.addData("position: ",  "LEFT");
                telemetry.update();
            }
        }
    }
}
