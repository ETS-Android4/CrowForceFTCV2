package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Toggle;

public class Arm {
    public Servo clampServo;
    public Servo elbowServo;

    public int elbowState = 0;

    private boolean heldElbowLast = false;

    private Toggle clampToggle = new Toggle(Toggle.ToggleTypes.flipToggle, true);

    public Arm(String _CLAMP_SERVO_CONFIG, String _ARM_SERVO_CONFIG) {
        clampServo = hardwareMap.servo.get(_CLAMP_SERVO_CONFIG);
        elbowServo = hardwareMap.servo.get(_ARM_SERVO_CONFIG);
    }

    public void init() {
        elbowServo.setPosition(0.3);
        clampServo.setPosition(1);
    }

    public void update() {
        if (gamepad2.left_bumper || gamepad2.right_bumper) {
            if (!heldElbowLast) {
                heldElbowLast = true;

                if (gamepad2.left_bumper && elbowState != 0) {
                    elbowState--;
                    telemetry.addData("lower", "elbow");
                }
                if (gamepad2.right_bumper && elbowState != 9) {
                    elbowState++;
                    telemetry.addData("raise", "elbow");
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

        telemetry.addData(" "  + elbowServo.getPosition(), "elbow");
        //robot.telemetry.update();
        clampToggle.updateToggle(gamepad2.b);

        if (clampToggle.getCurrentState()) {
            clampServo.setPosition(1);
        }
        else {
            clampServo.setPosition(0.5);
        }
    }
}
