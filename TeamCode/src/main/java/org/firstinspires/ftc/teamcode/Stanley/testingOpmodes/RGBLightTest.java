package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class RGBLightTest extends LinearOpMode {
    Servo servo = null;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "flywheelIndicator");

        double change = 0.1;
        double targetSpeed = 0;
        boolean correctingtoggle = false;

        waitForStart();

        while (opModeIsActive()) {

//            // ── Capture offset the first time the touch sensor fires ──
//            if (input.isPressed() && !offsetCaptured) {
//                encoderOffset = encoder.getCurrentPosition();
//                offsetCaptured = true;
//            }
//            // Position relative to the home/touch-sensor zero point
//            int relativePosition = encoder.getCurrentPosition() - encoderOffset;
            // ──────────────────────────────────────────────────────────

            if (gamepad1.yWasPressed()) {
                correctingtoggle = !correctingtoggle;
            }

            // Shift speed increment
            if (gamepad1.rightBumperWasPressed()) {
                change *= 10;
            } else if (gamepad1.leftBumperWasPressed()) {
                change /= 10;
            }

            if (gamepad1.dpadUpWasPressed()) {
                targetSpeed += change;
            }
            if (gamepad1.dpadDownWasPressed()) {
                targetSpeed -= change;
            }

//            // Stop and home when touch sensor is hit
//            if (input.isPressed()) {
//                correctingtoggle = false;
//                servo.setPower(0);
//                // Re-capture offset every time we return home
//                encoderOffset = encoder.getCurrentPosition();
//            }
            if (correctingtoggle) {
                servo.setPosition(targetSpeed);
            } else {
                servo.setPosition(0);
            }

            // ── Telemetry ─────────────────────────────────────────────
            telemetry.addData("Change", change);
            telemetry.addData("Target", targetSpeed);
            telemetry.addData("Toggle", correctingtoggle);
            telemetry.update();
        }
    }
}