package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Baguette;

@TeleOp(name="MainTeleop",group = "comp")
public class MecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Make sure ID's match config
        /*Baguette.flm = hardwareMap.dcMotor.get("f_l_m");
        Baguette.blm = hardwareMap.dcMotor.get("b_l_m");
        Baguette.frm = hardwareMap.dcMotor.get("f_rm");
        Baguette.brm = hardwareMap.dcMotor.get("b_r_m");

        // Reverse the right side motors, left for neverest
        Baguette.frm.setDirection(DcMotorSimple.Direction.REVERSE);
        Baguette.brm.setDirection(DcMotorSimple.Direction.REVERSE);

        //declares robots imu and sets to radians
        Baguette.imu = hardwareMap.get(BNO055IMU.class, "imu");*/


        //^removed and put into initialize baguette

        Baguette.initializeBaguette();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            Baguette.update();

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

            Baguette.flm.setPower(fl);
            Baguette.blm.setPower(bl);
            Baguette.frm.setPower(fr);
            Baguette.brm.setPower(br);
        }
    }

    public static void setAllPowers(double _pow) {
        Baguette.flm.setPower(_pow);
        Baguette.blm.setPower(_pow);
        Baguette.frm.setPower(_pow);
        Baguette.brm.setPower(_pow);
    }
}