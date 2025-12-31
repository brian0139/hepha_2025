package org.firstinspires.ftc.teamcode.Alvin;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intakeTest {
    public DcMotor intake;
    public colorSensor intakeSensor;

    private double intakePower = 1.0;    // Forward power
    private double reversePower = -1.0;  // Reverse power
    private double stopPower = 0.0;      // Stop power

    public intakeTest(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeSensor = new colorSensor(hardwareMap, "intakeSensor");
        stop();
    }

    public void setPower(double power) {
        if (intake != null) {
            intake.setPower(power);
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
        if (intakeSensor == null) return false;
        // Check if either green (1) or purple (2) is detected
        return intakeSensor.getDetected() != 0;
    }

    /**
     * Call this repeatedly in your loop.
     * Runs intake motor until a pixel is detected, then stops.
     * @return true if pixel detected (and motor stopped), false if still intaking
     */
    public boolean intakeUntilPixel() {
        if (isPixelDetected()) {
            stop();
            return true;  // Pixel found, intake stopped
        } else {
            intake();
            return false; // No pixel yet, keep running
        }
    }

    public void setPowerLevels(double forwardPower, double reversePower) {
        this.intakePower = forwardPower;
        this.reversePower = reversePower;
    }

    public void setColorThresholds(float greenMin, float greenMax,
                                   float purpleMin, float purpleMax) {
        if (intakeSensor != null) {
            intakeSensor.setThresholds(greenMin, greenMax, purpleMin, purpleMax);
        }
    }

    public void enableColorLight(boolean on) {
        if (intakeSensor != null) {
            intakeSensor.enableLight(on);
        }
    }
}