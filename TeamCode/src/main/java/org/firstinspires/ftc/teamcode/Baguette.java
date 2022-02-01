package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.DuckSpinner;

public class Baguette {
    // Drive Train
    public static DcMotor frm;
    public static DcMotor flm;
    public static DcMotor brm;
    public static DcMotor blm;

    // Subsystems
    public static DuckSpinner duckSpinner;
    public static Arm arm;

    // IMU
    public static BNO055IMU imu;

    public static void initializeBaguette() {
         duckSpinner = new DuckSpinner("spin_motor");
         arm = new Arm("clamp_s", "big_s");
    }

    public static void update() {
        arm.update();
        duckSpinner.update();
    }
}
