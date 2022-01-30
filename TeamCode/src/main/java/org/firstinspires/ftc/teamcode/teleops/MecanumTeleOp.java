package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="MainTeleop",group = "comp")
public class MecanumTeleOp extends LinearOpMode {
    public static DcMotor flm;
    public static DcMotor blm;
    public static DcMotor frm;
    public static DcMotor brm;

    public static BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        // Make sure ID's match config
        flm = hardwareMap.dcMotor.get("f_l_m");
        blm = hardwareMap.dcMotor.get("b_l_m");
        frm = hardwareMap.dcMotor.get("f_rm");
        brm = hardwareMap.dcMotor.get("b_r_m");

        // Reverse the right side motors, left for neverest
        frm.setDirection(DcMotorSimple.Direction.REVERSE);
        brm.setDirection(DcMotorSimple.Direction.REVERSE);

        //declares robots imu and sets to radians
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //
            //MECANUM DRIVE SECTION
            //
            double y = -gamepad1.left_stick_y; // note this is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            //Dividing by denominator will restrict domain of power to [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double fl = (y + x + rx) / denominator;
            double bl = (y - x + rx) / denominator;
            double fr = (y - x - rx) / denominator;
            double br = (y + x - rx) / denominator;

            flm.setPower(fl);
            blm.setPower(bl);
            frm.setPower(fr);
            brm.setPower(br);
        }
    }

    public static void setAllPowers(double _pow) {
        flm.setPower(_pow);
        blm.setPower(_pow);
        frm.setPower(_pow);
        brm.setPower(_pow);
    }
}