package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Baguette;
import org.firstinspires.ftc.teamcode.Toggle;

public class Arm {
    public Servo clampServo;
    public Servo elbowServo;

    public int elbowState = 0;

    private boolean heldElbowLast = false;

    private Toggle clampToggle = new Toggle(Toggle.ToggleTypes.flipToggle, true);

    public Arm(String _CLAMP_SERVO_CONFIG, String _ARM_SERVO_CONFIG) {
        clampServo = Baguette.hardwareMap.servo.get(_CLAMP_SERVO_CONFIG);
        elbowServo = Baguette.hardwareMap.servo.get(_ARM_SERVO_CONFIG);

        init();
    }

    public void init() {
        elbowServo.setPosition(0.3);
        clampServo.setPosition(1);
    }

    public void update() {
        if (Baguette.gamepad2.left_bumper || Baguette.gamepad2.right_bumper) {
            if (!heldElbowLast) {
                heldElbowLast = true;

                if (Baguette.gamepad2.left_bumper && elbowState != 0) {
                    elbowState--;
                    Baguette.telemetry.addData("lower", "elbow");
                }
                if (Baguette.gamepad2.right_bumper && elbowState != 9) {
                    elbowState++;
                    Baguette.telemetry.addData("raise", "elbow");
                }
            }
        }
        else {
            heldElbowLast = false;
        }


        if (elbowState == 0) {
            elbowServo.setPosition(0.2);
        }
        else if (elbowState >= 4) {
            elbowServo.setPosition((elbowState * (.1/7) + 0.63));
        }
        else {
            elbowServo.setPosition((elbowState * (0.15/7)) + 0.425);
        }

        Baguette.telemetry.addData(" "  + elbowServo.getPosition(), "elbow");
        Baguette.telemetry.update();

        clampToggle.updateToggle(Baguette.gamepad2.b);

        if (clampToggle.getCurrentState()) {
            clampServo.setPosition(1);
        }
        else {
            clampServo.setPosition(0.5);
        }
    }
}
