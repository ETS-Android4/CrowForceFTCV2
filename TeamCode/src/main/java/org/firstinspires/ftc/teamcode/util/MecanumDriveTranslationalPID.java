package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Math.PI;

import android.graphics.Point;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.teleops.MecanumTeleOp;

public class MecanumDriveTranslationalPID {
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static double thresholdInches = 1; // The accuracy to which move

    public static Pose2D lastPose; // the angle from the last frame. used in derivative calculations and wrap around errors
    public static Pose2D dPose; // difference between the angle from the current frame and lastAngle

    public static void turnPID(Pose2D _endPose) {
        Pose2D _startingAngle =

        lastAngle = _startingAngle;
        deltaTheta = 0;

        double error;
        double adjustedAngle;

        double integral = 0;

        double lastTime = System.nanoTime();
        double dt = 0;

        while (thresholdRadians < Math.abs(_startingAngle - _targetRadians)) {

            error = -_targetRadians - getCurrentAngle();

            deltaTheta = getCurrentAngle() - lastAngle;

            telemetry.addData("Current: ", getCurrentAngle());
            telemetry.addData("dTheta: ",  deltaTheta);
            telemetry.addData("Error:", error);
            telemetry.update();

            integral += error;

            dt = System.nanoTime() - lastTime;

            double P = kP * error;
            double I = kI * integral;
            double D = kD * (error - lastAngle) / dt;

            MecanumTeleOp.setAllPowers(P+I+D);

            lastTime = System.nanoTime();
        }

        MecanumTeleOp.setAllPowers(0);
    }

    public static Pose2D getCurrentPose() {
        Position pos = MecanumTeleOp.imu.getPosition().toUnit(DistanceUnit.INCH);
        return new Pose2D(pos.x, pos.y, MecanumTeleOp.imu.)
    }
}
