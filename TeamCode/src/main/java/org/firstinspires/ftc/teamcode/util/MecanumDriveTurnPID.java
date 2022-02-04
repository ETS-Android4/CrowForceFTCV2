package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import static java.lang.Math.PI;

import android.icu.text.SymbolTable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Baguette;
import org.firstinspires.ftc.teamcode.teleops.MecanumTeleOp;

public class MecanumDriveTurnPID {

    public static final double kP = 1.7;
    public static final double kI = 0;
    public static final double kD = 0;

    public static double thresholdDegrees = 2; // The accuracy to which the robot will turn

    public static double lastAngle; // the angle from the last frame. used in derivative calculations and wrap around errors
    public static double deltaTheta; // difference between the angle from the current frame and lastAngle


    //LEFT: + : 0 -> 180 -> -180 -> 0
    //Right: - : 0 -> -180 -> 180 -> 0
    public static void turnPID(double _targetDegrees) {
        double _startingAngle = getCurrentAngle();

        lastAngle = _startingAngle;
        deltaTheta = 0;

        double error = Double.MAX_VALUE;
        double current = _startingAngle;

        double integral = 0;

        double lastTime = System.nanoTime();
        double dt = 0;

        while (thresholdDegrees < Math.abs(current - _targetDegrees)) {

            current = getCurrentAngle();
            error = _targetDegrees - current;

            deltaTheta = current - lastAngle;


            Baguette.telemetry.addData("Real: ", Baguette.realAngle);
            Baguette.telemetry.addData("dTheta: ",  deltaTheta);
            Baguette.telemetry.addData("Error:", error);


            dt = System.nanoTime() - lastTime;
            integral += error * dt;

            double P = kP * error;
            double I = kI * integral;
            double D = kD * (error - lastAngle) / dt;

            double PID = P+I+D;

            PID *= (1.0/40);

            Baguette.telemetry.addData("D Adjusted:", (error - lastAngle) / dt);
            Baguette.telemetry.addData("PID Adjusted:", PID);


            Baguette.frm.setPower(-PID);
            Baguette.flm.setPower(PID);
            Baguette.brm.setPower(-PID);
            Baguette.blm.setPower(PID);

            lastAngle = current;
            lastTime = System.nanoTime();

            Baguette.telemetry.addData("condition", Math.abs(current - _targetDegrees));
            Baguette.telemetry.update();
        }

        Baguette.mecanumDrive.setAllPowers(0);
    }

    public static double getCurrentAngle() {
        return Baguette.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle;
    }
}
