package org.firstinspires.ftc.teamcode.Alvin;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;



public class intake {

    private Servo intakeServo;
    private colorSensor colorDetector;
    private double intakePower = 1.0;    // Forward power
    private double reversePower = -1.0;  // Reverse power
    private double stopPower = 0.0;      // Stop power

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
        return colorDetector.getDetected() != 0;
    }

    public boolean intakeUntilPixel(long timeoutMs) {
        long startTime = System.currentTimeMillis();
        intake();  // Start intaking

        while (!isPixelDetected() && (System.currentTimeMillis() - startTime < timeoutMs)) {
            // Continue intakin
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        stop();
        return isPixelDetected();
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