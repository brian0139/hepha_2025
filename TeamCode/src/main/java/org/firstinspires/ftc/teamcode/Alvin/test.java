package org.firstinspires.ftc.teamcode.Alvin;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Color Sensor Test", group = "Test")
public class test extends OpMode {

    private colorSensor color;

    @Override
    public void init() {
        color = new colorSensor(hardwareMap, "colorSensor");

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        int detected = color.getDetected();

        String result;
        if (detected == 1) {
            result = "GREEN";
        } else if (detected == 2) {
            result = "PURPLE";
        } else {
            result = "NONE";
        }

        telemetry.addData("Detected", result);

        float[] hsv = color.readHSV();
        telemetry.addData("Hue", hsv[0]);
        telemetry.addData("Sat", hsv[1]);
        telemetry.addData("Val", hsv[2]);

        telemetry.update();
    }
}
