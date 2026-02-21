package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

@TeleOp
public class offsetAvg extends LinearOpMode {
    TouchSensor input = null;
    CRServo servo = null;
    DcMotorEx encoder = null;
    FtcDashboard dashboard = null;
    Telemetry dashboardTelemetry = null;
    long slot1Total=0;
    long slot2Total=0;
    long slot3Total=0;
    int slot1Cnt=0;
    int slot2Cnt=0;
    int slot3Cnt=0;
    int totalCnt=0;

    @Override
    public void runOpMode() {
        // Initialize hardware
        input = hardwareMap.get(TouchSensor.class, "touch");
        servo = hardwareMap.get(CRServo.class, "spindexerServo");
        encoder = hardwareMap.get(DcMotorEx.class, "intake");
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        double change = 0.1;
        double targetSpeed = 0;
        boolean running = false;

        // Track previous touch state for edge detection (log only on new press)
        boolean lastTouchState = false;

        // Store encoder positions each time the touch sensor is triggered
        ArrayList<Integer> encoderLog = new ArrayList<>();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.log().setCapacity(50);
            dashboardTelemetry.log().setCapacity(50);
            // --- Speed Adjustment ---
            if (gamepad1.rightBumperWasPressed()) {
                change *= 10;
            } else if (gamepad1.leftBumperWasPressed()) {
                change /= 10;
            }

            if (gamepad1.dpadUpWasPressed()) {
                targetSpeed += change;
                targetSpeed = Math.min(targetSpeed, 1.0); // Clamp to valid servo range
            }
            if (gamepad1.dpadDownWasPressed()) {
                targetSpeed -= change;
                targetSpeed = Math.max(targetSpeed, -1.0); // Clamp to valid servo range
            }

            // --- Toggle Spindexer On/Off with Y Button ---
            if (gamepad1.yWasPressed()) {
                running = !running;
            }

            if (gamepad1.xWasPressed()){
                encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            // Apply power based on toggle state
            servo.setPower(running ? targetSpeed : -gamepad1.left_stick_x);

            // --- Touch Sensor: Log encoder position on each new press ---
            boolean currentTouchState = input.isPressed();
            if (currentTouchState && !lastTouchState) {
                // Rising edge — sensor was just pressed
                int pos = encoder.getCurrentPosition();
                totalCnt++;
                if (totalCnt%3==1){
                    slot1Cnt++;
                    slot1Total+=pos;
                }else if (totalCnt%3==2){
                    slot2Cnt++;
                    slot2Total+=pos;
                }else{
                    slot3Cnt++;
                    slot3Total+=pos;
                }
            }
            lastTouchState = currentTouchState;

            // --- Telemetry ---
            telemetry.addData("Running", running);
            telemetry.addData("Target Speed", targetSpeed);
            telemetry.addData("Change Step", change);
            telemetry.addData("Encoder Ticks", encoder.getCurrentPosition());
            telemetry.addData("Touch Pressed", input.isPressed());
            telemetry.addData("Total Triggers", totalCnt);
            telemetry.addData("Slot 1 Avg",slot1Total/slot1Cnt);
            telemetry.addData("Slot 1 Count",slot1Cnt);
            telemetry.addData("Slot 2 Avg",slot2Total/slot2Cnt);
            telemetry.addData("Slot 2 Count",slot2Cnt);
            telemetry.addData("Slot 3 Avg",slot3Total/slot3Cnt);
            telemetry.addData("Slot 3 Count",slot3Cnt);

            // Show last few logged positions
            int logSize = encoderLog.size();
//            for (int i = Math.max(0, logSize - 5); i < logSize; i++) {
//                telemetry.addData("Log[" + i + "]", encoderLog.get(i));
//            }

            telemetry.update();
//
//            // Dashboard telemetry
//            dashboardTelemetry.addData("Running", running);
//            dashboardTelemetry.addData("Target Speed", targetSpeed);
//            dashboardTelemetry.addData("Encoder Ticks", encoder.getCurrentPosition());
//            dashboardTelemetry.addData("Touch Pressed", input.isPressed());
//            dashboardTelemetry.addData("PressedNum", input.isPressed() ? 1 : 0);
//            dashboardTelemetry.addData("Total Triggers", encoderLog.size());
//            if (logSize > 0) {
//                dashboardTelemetry.addData("Last Logged Position", encoderLog.get(logSize - 1));
//            }
//            dashboardTelemetry.update();
        }
    }
}