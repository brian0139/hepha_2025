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

    private float GREEN_H_MIN = 85f;
    private float GREEN_H_MAX = 170f;
    private float PURPLE_H_MIN = 260f;
    private float PURPLE_H_MAX = 320f;
    private float MIN_SAT = 0.25f; // raise (e.g., 0.35–0.45) if colors look washed out
    private float MIN_VAL = 0.10f;

    public void init(HardwareMap hardwareMap, String deviceName) {
        sensor = hardwareMap.get(NormalizedColorSensor.class, deviceName);
        enableLight(true);
    }

    public float[] readHSV() {
        if (sensor == null) return new float[]{0f, 0f, 0f};
        NormalizedRGBA n = sensor.getNormalizedColors();
        Color.colorToHSV(n.toColor(), hsv);
        return hsv;
    }

    public boolean isGreen() {
        readHSV();
        return hsv[1] >= MIN_SAT && hsv[2] >= MIN_VAL && inRange(hsv[0], GREEN_H_MIN, GREEN_H_MAX);
    }

    public boolean isPurple() {
        readHSV();
        return hsv[1] >= MIN_SAT && hsv[2] >= MIN_VAL && inRange(hsv[0], PURPLE_H_MIN, PURPLE_H_MAX);
    }

    public Detected getDetected() {
        if (isGreen())  return Detected.GREEN;
        if (isPurple()) return Detected.PURPLE;
        return Detected.NONE;
    }

    public void enableLight(boolean on) {
        if (sensor instanceof SwitchableLight) {
            ((SwitchableLight) sensor).enableLight(on);
        }
    }


    public void setThresholds(float greenMin, float greenMax,
                              float purpleMin, float purpleMax,
                              float minSat, float minVal) {
        GREEN_H_MIN = greenMin; GREEN_H_MAX = greenMax;
        PURPLE_H_MIN = purpleMin; PURPLE_H_MAX = purpleMax;
        MIN_SAT = minSat; MIN_VAL = minVal;
    }

    private boolean inRange(float v, float lo, float hi) { return v >= lo && v <= hi; }
}
