package org.firstinspires.ftc.teamcode.Alvin;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

public class colorSensor {

    public enum Detected { GREEN, PURPLE, NONE }

    private NormalizedColorSensor sensor;
    private final float[] hsv = new float[3];

    // Color threshold values
    private float GREEN_H_MIN = 150f;
    private float GREEN_H_MAX = 180f;
    private float PURPLE_H_MIN = 210f;
    private float PURPLE_H_MAX = 245f;
    private float MIN_SAT = 0.25f; // Adjust to prevent washed-out colors
 // Adjust for low light sensitivity
    /**
     * Initializes the color sensor and enables the light for better detection.
     *
     * @param hardwareMap   HardwareMap for accessing sensors
     * @param deviceName    Name of the color sensor in the hardware configuration
     */
    public colorSensor(HardwareMap hardwareMap, String deviceName) {
        sensor = hardwareMap.get(NormalizedColorSensor.class, deviceName);
        enableLight(true); // Turn on the light to help with color detection
    }

    /**
     * Reads the current color detected by the color sensor and returns it as HSV values.
     *
     * @return float[] Array containing HSV values: [H, S, V]
     */
    public float[] readHSV() {
//        if (sensor == null) return new float[]{0f, 0f, 0f}; // Return default if sensor is not initialized
        NormalizedRGBA n = sensor.getNormalizedColors();
        Color.colorToHSV(n.toColor(), hsv); // Convert the color to HSV
        return hsv;
    }

    /**
     * Checks if the detected color is within the green color range.
     *
     * @return true if the detected color is green, false otherwise
     */
    public boolean isGreen() {
        readHSV();
        return inRange(hsv[0], GREEN_H_MIN, GREEN_H_MAX);
    }

    /**
     * Checks if the detected color is within the purple color range.
     *
     * @return true if the detected color is purple, false otherwise
     */
    public boolean isPurple() {
        readHSV();
        return inRange(hsv[0], PURPLE_H_MIN, PURPLE_H_MAX);
    }

    public boolean isWall(){
        readHSV();
        return inRange(hsv[0], 40, 44);
    }

    /**
     * Returns the detected color as an integer.
     *
     * @return 0 if no color detected, 1 for green, 2 for purple
     */
    public int getDetected() {
        int tmp = 0; // Default is no color detected
        if (isGreen()) tmp = 1;  // Green detected
        if (isPurple()) tmp = 2; // Purple detected
        return tmp;
    }

    /**
     * Enables or disables the sensor's light.
     *
     * @param on   true to enable light, false to disable
     */
    public void enableLight(boolean on) {
        if (sensor instanceof SwitchableLight) {
            ((SwitchableLight) sensor).enableLight(on);
        }
    }

    /**
     * Sets custom thresholds for green and purple detection, as well as saturation and value for color detection.
     *
     * @param greenMin      Minimum hue for green detection
     * @param greenMax      Maximum hue for green detection
     * @param purpleMin     Minimum hue for purple detection
     * @param purpleMax     Maximum hue for purple detection
     */
    public void setThresholds(float greenMin, float greenMax,
                              float purpleMin, float purpleMax) {
        GREEN_H_MIN = greenMin; GREEN_H_MAX = greenMax;
        PURPLE_H_MIN = purpleMin; PURPLE_H_MAX = purpleMax;
    }

    /**
     * Helper method to check if a value is within a specified range.
     *
     * @param v   The value to check
     * @param lo  The lower bound of the range
     * @param hi  The upper bound of the range
     * @return    true if v is between lo and hi (inclusive), false otherwise
     */
    private boolean inRange(float v, float lo, float hi) {
        return v >= lo && v <= hi;
    }

    public void adjustColorThreshold(String colorName, float minHue, float maxHue) {
        if (colorName.equalsIgnoreCase("green")) {
            GREEN_H_MIN = minHue;
            GREEN_H_MAX = maxHue;
        } else if (colorName.equalsIgnoreCase("purple")) {
            PURPLE_H_MIN = minHue;
            PURPLE_H_MAX = maxHue;
        }
    }
}
