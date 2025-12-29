package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class analoginputtest extends LinearOpMode {
    AnalogInput input=null;
    CRServo servo=null;
    @Override
    public void runOpMode(){
        input=hardwareMap.get(AnalogInput.class,"hoodAnalog");
        servo=hardwareMap.get(CRServo.class,"hoodServo");
        waitForStart();
        while(opModeIsActive()){
            servo.setPower(-gamepad1.left_stick_y);
            telemetry.addData("Voltage",input.getVoltage());
            telemetry.update();
        }
    }
}
