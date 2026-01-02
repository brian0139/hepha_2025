package org.firstinspires.ftc.teamcode.Alvin;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intake {
    public DcMotor intakeMotor;
    public colorSensor colorDetector;

    private double intakePower = 1.0;    // Forward power
    private double reversePower = -1.0;  // Reverse power
    private double stopPower = 0.0;      // Stop power

    //non-blocking "intake until pixel"
    private boolean runningPixelIntake = false;

    public intake(HardwareMap hardwareMap, String servoDeviceName, String colorDeviceName) {
        intakeMotor = hardwareMap.get(DcMotor.class, servoDeviceName);
        colorDetector = new colorSensor(hardwareMap, colorDeviceName);
        colorDetector.enableLight(true);
        stop();  // Ensure motor is stopped initially
    }

    public void setPower(double power) {
        if (intakeMotor != null) {
            intakeMotor.setPower(power);
        }
    }

    public void intake() {
        setPower(intakePower);
    }

    public void reverse() {
        setPower(reversePower);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }

    // Check if a pixel is detected (green or purple)
    public boolean isPixelDetected() {
        if (colorDetector == null) return false;
        return colorDetector.getDetected() != 0;
    }

    /**
     * Runs intake motor until a pixel is detected, then stops the motor.
     * @return true once a pixel is detected.
     */
    public boolean intakeUntilPixel() {
        //Start intaking
        if (!runningPixelIntake){
            intake();
            runningPixelIntake=true;
        }
        // Check for pixel detection
        if (isPixelDetected()) {
            stop();  // Stop the intake motor as soon as a pixel is detected
            runningPixelIntake = false;  // Stop the routine
            return true;  // Pixel detected
        }

        // Continue running intake motor if no pixel is detected yet
        return false;  // No pixel yet, still running
    }

    public void setPowerLevels(double forwardPower, double reversePower) {
        this.intakePower = forwardPower;
        this.reversePower = reversePower;
    }

    public void setColorThresholds(float greenMin, float greenMax,
                                   float purpleMin, float purpleMax,
                                   float minSat, float minVal) {
        if (colorDetector != null) {
            colorDetector.setThresholds(greenMin, greenMax, purpleMin, purpleMax);
        }
    }

    public void enableColorLight(boolean on) {
        if (colorDetector != null) {
            colorDetector.enableLight(on);
        }
    }
}
