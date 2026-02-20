package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class spindexerTouchTest extends LinearOpMode {
    TouchSensor input = null;
    CRServo servo = null;
    DcMotorEx encoder = null;
    FtcDashboard dashboard = null;
    Telemetry dashboardTelemetry = null;

    @Override
    public void runOpMode() {
        input = hardwareMap.get(TouchSensor.class, "touch");
        servo = hardwareMap.get(CRServo.class, "spindexerServo");
        encoder = hardwareMap.get(DcMotorEx.class, "intake");

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        // ── Offset setup ──────────────────────────────────────────────
        // We store the raw tick value at the moment the touch sensor
        // is first triggered so all subsequent readings are relative
        // to that home position.
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int encoderOffset = 0;
        boolean offsetCaptured = false;
        // ──────────────────────────────────────────────────────────────

        double change = 0.1;
        double targetSpeed = 0;
        boolean correctingtoggle = false;

        waitForStart();

        while (opModeIsActive()) {

            // ── Capture offset the first time the touch sensor fires ──
            if (input.isPressed() && !offsetCaptured) {
                encoderOffset = encoder.getCurrentPosition();
                offsetCaptured = true;
            }
            // Position relative to the home/touch-sensor zero point
            int relativePosition = encoder.getCurrentPosition() - encoderOffset;
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

            // Stop and home when touch sensor is hit
            if (input.isPressed()) {
                correctingtoggle = false;
                servo.setPower(0);
                // Re-capture offset every time we return home
                encoderOffset = encoder.getCurrentPosition();
            } else if (correctingtoggle) {
                servo.setPower(targetSpeed);
            } else {
                servo.setPower(0);
            }

            // ── Telemetry ─────────────────────────────────────────────
            telemetry.addData("Change", change);
            telemetry.addData("Target Speed", targetSpeed);
            telemetry.addData("Holding", correctingtoggle);
            telemetry.addData("Pressed", input.isPressed());
            telemetry.addData("Raw Encoder Ticks", encoder.getCurrentPosition());
            telemetry.addData("Encoder Offset", encoderOffset);
            telemetry.addData("Relative Position", relativePosition);  // ← key value
            telemetry.addData("Offset Captured", offsetCaptured);
            telemetry.update();

            dashboardTelemetry.addData("PressedNum", input.isPressed() ? 1 : 0);
            dashboardTelemetry.addData("Pressed", input.isPressed());
            dashboardTelemetry.addData("Raw Ticks", encoder.getCurrentPosition());
            dashboardTelemetry.addData("Relative Position", relativePosition);
            dashboardTelemetry.addData("Encoder Offset", encoderOffset);
            dashboardTelemetry.update();
        }
    }
}