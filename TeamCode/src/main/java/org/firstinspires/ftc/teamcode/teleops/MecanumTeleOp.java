package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Baguette;

@TeleOp(name="MainTeleop",group = "comp")
public class MecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Baguette.initializeBaguette(this);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            Baguette.update();
            Baguette.telemetry.addData("Pos X:", Baguette.imu.getPosition().toUnit(DistanceUnit.INCH).x);
            Baguette.telemetry.addData("Pos Y:", Baguette.imu.getPosition().toUnit(DistanceUnit.INCH).y);
            Baguette.telemetry.addData("Angle Degrees:", Baguette.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle);
            Baguette.telemetry.update();

        }
    }
}