package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Baguette;
import org.firstinspires.ftc.teamcode.Wait;

public class DuckSpinner {
    public DcMotor spinMotor;

    public DuckSpinner (String _SPIN_MOTOR_CONFIG) {
        spinMotor = Baguette.hardwareMap.dcMotor.get(_SPIN_MOTOR_CONFIG);

        init();
    }

    public void spinMotorForTime(double _power, long _time) {
        spinMotor.setPower(_power);
        Wait.waitTime(1500);
        spinMotor.setPower(0);
    }

    public void setSpinMotor(double _power) {
        spinMotor.setPower(_power);
    }

    public void init() {

    }

    public void update() {
        double power = 0;

        if (Baguette.gamepad2.a) power += 0.25;
        if (Baguette.gamepad2.y) power -= 0.25;

        setSpinMotor(power);
    }
}