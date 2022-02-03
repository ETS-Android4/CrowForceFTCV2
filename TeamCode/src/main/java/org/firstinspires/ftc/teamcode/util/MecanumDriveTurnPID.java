package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import static java.lang.Math.PI;

import android.icu.text.SymbolTable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Baguette;
import org.firstinspires.ftc.teamcode.teleops.MecanumTeleOp;

public class MecanumDriveTurnPID {

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static double thresholdRadians = PI/90; // The accuracy to which the robot will turn

    public static double lastAngle; // the angle from the last frame. used in derivative calculations and wrap around errors
    public static double deltaTheta; // difference between the angle from the current frame and lastAngle

    public static void turnPID(double _targetRadians) {
        double _startingAngle = getCurrentAngle();

        lastAngle = _startingAngle;
        deltaTheta = 0;

        double error;
        double adjustedAngle;

        double integral = 0;

        double lastTime = System.nanoTime();
        double dt = 0;

        while (thresholdRadians < Math.abs(_startingAngle - _targetRadians)) {

            error = _targetRadians - getCurrentAngle();

            deltaTheta = getCurrentAngle() - lastAngle;

            telemetry.addData("Current: ", getCurrentAngle());
            telemetry.addData("dTheta: ",  deltaTheta);
            telemetry.addData("Error:", error);
            telemetry.update();



            dt = System.nanoTime() - lastTime;
            integral += error * dt;

            double P = kP * error;
            double I = kI * integral;
            double D = kD * (error - lastAngle) / dt;

            double PID = P+I+D;

            Baguette.frm.setPower(PID);
            Baguette.flm.setPower(-PID);
            Baguette.brm.setPower(-PID);
            Baguette.blm.setPower(PID);

            lastTime = System.nanoTime();
        }

        MecanumTeleOp.setAllPowers(0);
    }

    public static double getCurrentAngle() {
        return Baguette.imu.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle;
    }
}