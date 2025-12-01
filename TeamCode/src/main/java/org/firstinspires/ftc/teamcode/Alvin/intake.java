package org.firstinspires.ftc.teamcode.Alvin;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class intake {
    private Servo intakeServo;
    private colorSensor colorDetector;

    private double intakePower = 1.0;    // Forward power
    private double reversePower = -1.0;  // Reverse power
    private double stopPower = 0.0;      // Stop power

    //non-blocking "intake until pixel
    private boolean runningPixelIntake = false;
    private long pixelIntakeStartTime = 0;
    private long pixelIntakeTimeoutMs = 0;

    public intake(HardwareMap hardwareMap, String servoDeviceName, String colorDeviceName) {
        intakeServo = hardwareMap.get(Servo.class, servoDeviceName);
        colorDetector = new colorSensor();
        colorDetector.init(hardwareMap, colorDeviceName);
        colorDetector.enableLight(true);
        stop();
    }

    public void setPower(double power) {
        if (intakeServo != null) {
            // -1.0 power to 0.0 position (full reverse)
            //  0.0 power to 0.5 position (stop)
            //  1.0 power to 1.0 position (full forward)
            double servoPosition = (power + 1.0) / 2.0;
            intakeServo.setPosition(servoPosition);
        }
    }

    public void intake() {
        setPower(intakePower);
    }

    public void reverse() {
        setPower(reversePower);
    }

    public void stop() {
        setPower(stopPower);
    }

    public boolean isPixelDetected() {
        if (colorDetector == null) return false;
        return colorDetector.getDetected() != colorSensor.Detected.NONE;
    }

    /**
     * First call: starts the intake and records start time.
     * Later calls: checks for pixel or timeout, then stops.
     * @param timeoutMs how long to try, in milliseconds
     * @return true once a pixel is detected OR timeout reached.
     */
    public boolean intakeUntilPixel(long timeoutMs) {
        // If we rnt currently running this routine, start it
        if (!runningPixelIntake) {
            runningPixelIntake = true;
            pixelIntakeStartTime = System.currentTimeMillis();
            pixelIntakeTimeoutMs = timeoutMs;
            intake();
        }

        // Alr running: check conditions once per call
        boolean timedOut =
                System.currentTimeMillis() - pixelIntakeStartTime >= pixelIntakeTimeoutMs;

        if (isPixelDetected() || timedOut) {
            stop();
            runningPixelIntake = false;
            // If timed out and no pixel, still return false for "detected"
            return isPixelDetected();
        }

        // Still running, nothing finished yet
        return false;
    }

    public void setPowerLevels(double forwardPower, double reversePower) {
        this.intakePower = forwardPower;
        this.reversePower = reversePower;
    }

    public void setColorThresholds(float greenMin, float greenMax,
                                   float purpleMin, float purpleMax,
                                   float minSat, float minVal) {
        if (colorDetector != null) {
            colorDetector.setThresholds(greenMin, greenMax, purpleMin, purpleMax, minSat, minVal);
        }
    }

    public void enableColorLight(boolean on) {
        if (colorDetector != null) {
            colorDetector.enableLight(on);
        }
    }
}
