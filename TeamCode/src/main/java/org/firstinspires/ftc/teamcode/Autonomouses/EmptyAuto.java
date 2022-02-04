package org.firstinspires.ftc.teamcode.Autonomouses;

import static org.firstinspires.ftc.teamcode.util.MecanumDriveTurnPID.getCurrentAngle;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Baguette;
import org.firstinspires.ftc.teamcode.Wait;
import org.firstinspires.ftc.teamcode.util.MecanumDriveTranslationalPID;
import org.firstinspires.ftc.teamcode.util.MecanumDriveTurnPID;
import org.firstinspires.ftc.teamcode.util.Pose2D;

@Autonomous(name="Angle Writer")
public class EmptyAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Baguette.initializeBaguette(this);
        waitForStart();
        double lastAngle = 0;
        double deltaTheta = 0;

        int lastMult360 = 0;



        if (opModeIsActive()) {
            /*double current = getCurrentAngle();

            deltaTheta = current - lastAngle;

            lastMult360 = (int)(Baguette.realAngle/360);

            if (deltaTheta >= 180) {
                deltaTheta -= 360;
                Baguette.realAngle = 360 * lastMult360;
            }

            else if (deltaTheta <= -180) {
                deltaTheta += 360;
            }


            Baguette.realAngle += deltaTheta;

            Baguette.telemetry.addData("Real: ", Baguette.realAngle);
            Baguette.telemetry.addData("imu angle: ", current);
            Baguette.telemetry.addData("dTheta: ",  deltaTheta);

            Baguette.telemetry.update();

            lastAngle = getCurrentAngle();*/

            MecanumDriveTurnPID.turnPID(90);

            Wait.waitTime(1000);


            MecanumDriveTranslationalPID.translatePID(new Pose2D(0, 24, 0));
        }
    }
}
