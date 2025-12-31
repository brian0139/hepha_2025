package org.firstinspires.ftc.teamcode.Alvin;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="Simple IntakeTest", group="Test")
public class intakeTest extends LinearOpMode {

    // Constants for hardware configuration
    private static final String INTAKE_MOTOR_NAME = "intakeMotor";
    private static final String COLOR_SENSOR_NAME = "colorSensor";

    @Override
    public void runOpMode() {
        intake in = new intake(hardwareMap, INTAKE_MOTOR_NAME, COLOR_SENSOR_NAME);

        boolean runningUntilPixel = false;
        boolean pixelDetected = false;

        telemetry.addLine("IntakeTest ready.");
        telemetry.addLine("A: intake (forward)   B: reverse   X: stop");
        telemetry.addLine("Y: start/stop intakeUntilPixel()");
        telemetry.addLine("Controls: A=forward, B=reverse, X=stop, Y=start/stop Pixel Test");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;

            // Handle button presses
            if (a) {
                in.intake();  // Start intake forward
                runningUntilPixel = false;
            }
            if (b) {
                in.reverse();  // Reverse intake
                runningUntilPixel = false;
            }
            if (x) {
                in.stop();  // Stop intake
                runningUntilPixel = false;
            }

            // Start/stop intakeUntilPixel process when Y is pressed
            if (y) {
                runningUntilPixel = !runningUntilPixel; // Toggle
                pixelDetected = false; // Reset detection flag
                if (runningUntilPixel) {
                    telemetry.addLine("Running intakeUntilPixel...");
                } else {
                    telemetry.addLine("Stopped intakeUntilPixel.");
                }
            }

            // Run the intake until pixel (or color 1/2) is detected
            if (runningUntilPixel) {
                pixelDetected = in.intakeUntilPixel();
            }

            // Show the results
            telemetry.addData("PixelDetected", pixelDetected ? "Yes" : "No");
            telemetry.addData("Detected",in.colorDetector.getDetected());
            telemetry.addLine("Controls:");
            telemetry.addLine("A: forward | B: reverse | X: stop | Y: toggle Pixel Test");
            telemetry.update();

            idle();
        }
    }
}
