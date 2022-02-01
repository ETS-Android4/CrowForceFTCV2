package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Math.PI;

import android.graphics.Point;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Baguette;
import org.firstinspires.ftc.teamcode.teleops.MecanumTeleOp;

public class MecanumDriveTranslationalPID {
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static double thresholdInchesSquared = 1; // The accuracy to which move. squared for efficiency

    public static Pose2D lastPose; // the angle from the last frame. used in derivative calculations and wrap around errors
    public static Pose2D dPose; // difference between the angle from the current frame and lastAngle

    public static void turnPID(Pose2D _endPose) {
        Pose2D startingPose = getCurrentPose();

        lastPose = startingPose;
        dPose = Pose2D.zero;

        Pose2D currentPos = startingPose;
        Pose2D error = _endPose.signedDifference(currentPos);
        double errorSqrMag = error.x * error.x + error.y * error.y;

        Pose2D integral = Pose2D.zero;

        double lastTime = System.nanoTime();
        double dt = 0;

        while (thresholdInchesSquared < errorSqrMag) {
            currentPos = getCurrentPose();

            error = _endPose.signedDifference(currentPos);

            dPose = currentPos.signedDifference(lastPose);

            telemetry.addData("Current: ", currentPos);
            telemetry.addData("dTheta: ",  dPose);
            telemetry.addData("Error:", error);
            telemetry.update();

            dt = System.nanoTime() - lastTime;

            integral = integral.add( error.mult(dt) ) ; // frame by frame riemann sum

            Pose2D P = error.mult(kP);
            Pose2D I = integral.mult(kI);
            Pose2D D = dPose.mult(error).mult(1/dt).mult(kD);

            Pose2D PID = P.add(I).add(D);

            //Dividing by denominator will restrict domain of power to [-1, 1]

            double denominator = Math.max(Math.abs(PID.y) + Math.abs(PID.x), 1);
            double y = PID.y;
            double x = PID.x;

            double fl = (y + x) / denominator;
            double bl = (y - x) / denominator;
            double fr = (y - x) / denominator;
            double br = (y + x) / denominator;

            Baguette.flm.setPower(fl);
            Baguette.blm.setPower(bl);
            Baguette.frm.setPower(fr);
            Baguette.brm.setPower(br);

            lastTime = System.nanoTime();
        }

        MecanumTeleOp.setAllPowers(0);
    }

    public static Pose2D getCurrentPose() {
        Position pos = Baguette.imu.getPosition().toUnit(DistanceUnit.INCH);
        return new Pose2D(pos.x, pos.y, Baguette.imu.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle);
    }
}
