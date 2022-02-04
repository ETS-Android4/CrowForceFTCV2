package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Baguette;

public class MecanumDrive {

    public MecanumDrive() {
        init();
    }

    public void init() {
        Baguette.flm = Baguette.hardwareMap.dcMotor.get("f_l_m");
        Baguette.blm = Baguette.hardwareMap.dcMotor.get("b_l_m");
        Baguette.frm = Baguette.hardwareMap.dcMotor.get("f_r_m");
        Baguette.brm = Baguette.hardwareMap.dcMotor.get("b_r_m");

        // Reverse the right side motors, left for neverest
        Baguette.flm.setDirection(DcMotorSimple.Direction.REVERSE);
        Baguette.blm.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update() {
        double y = Baguette.gamepad1.left_stick_y;
        double x = Baguette.gamepad1.left_stick_x;
        double rx = Baguette.gamepad1.right_stick_x;

        setMotorsFromInput(x, y, rx);
    }

    public static void setAllPowers(double _pow) {
        Baguette.flm.setPower(_pow);
        Baguette.blm.setPower(_pow);
        Baguette.frm.setPower(_pow);
        Baguette.brm.setPower(_pow);
    }

    public static void setMotorsFromInput(double x, double y, double rx) {
        //Dividing by denominator will restrict domain of power to [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double fl = (y + x - rx) / denominator;
        double bl = (y - x - rx) / denominator;
        double fr = (y - x + rx) / denominator;
        double br = (y + x + rx) / denominator;

        Baguette.flm.setPower(fl);
        Baguette.blm.setPower(bl);
        Baguette.frm.setPower(fr);
        Baguette.brm.setPower(br);
    }
}
