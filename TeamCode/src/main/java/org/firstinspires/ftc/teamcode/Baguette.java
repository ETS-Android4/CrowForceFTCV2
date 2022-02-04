package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.DuckSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.camera;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Baguette {
    // Drive Train
    public static DcMotor frm;
    public static DcMotor flm;
    public static DcMotor brm;
    public static DcMotor blm;

    // Subsystems
    public static MecanumDrive mecanumDrive;
    public static DuckSpinner duckSpinner;
    public static Arm arm;

    // IMU
    public static BNO055IMU imu;

    public static HardwareMap hardwareMap;

    public static Gamepad gamepad1;
    public static Gamepad gamepad2;

    public static Telemetry telemetry;

    public static OpenCvCamera webcam;
    public static camera pipeline;

    public static double realAngle = 0;

    public static void initializeBaguette(OpMode _opMode) {


        hardwareMap = _opMode.hardwareMap;

        gamepad1 = _opMode.gamepad1;
        gamepad2 = _opMode.gamepad2;

        telemetry = _opMode.telemetry;

        Baguette.imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.accelRange = BNO055IMU.AccelRange.G16;
        imu.initialize(params);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1);
        realAngle = 0;

        //initCamera(); Do this after init baguette, only in opencv enabled subsystems

        mecanumDrive = new MecanumDrive();
        duckSpinner = new DuckSpinner("spin_motor");
        arm = new Arm("clamp_s", "big_s");
    }

    public static void initCamera() {
        int cameraMonitorViewId = Baguette.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new camera();
        webcam.setPipeline(pipeline);
    }

    public static void update() {
        mecanumDrive.update();
        arm.update();
        duckSpinner.update();
    }
}