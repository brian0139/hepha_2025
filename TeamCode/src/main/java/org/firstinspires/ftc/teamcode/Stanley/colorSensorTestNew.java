package org.firstinspires.ftc.teamcode.Stanley;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Alvin.colorSensor;
import java.util.Vector;

@TeleOp

public class colorSensorTestNew extends LinearOpMode{
    CRServo spindexer;
    colorSensor colorsensoroperator;

    //main loop
    @Override
    public void runOpMode() {
        //initiate drivetrain motors
        spindexer=hardwareMap.get(CRServo.class,"spindexerServo");
        colorsensoroperator=new colorSensor(hardwareMap,"colorSensor");
        double spindexerPower=0;
        //telemetry message to signify robot waiting
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        Vector<String> detections=new Vector<>();
        //wait for driver to press play
        waitForStart();
        //repeat until opmode ends
        while (opModeIsActive()){
            if (gamepad1.yWasPressed()) spindexer.setPower(spindexerPower);
            if (gamepad1.dpadUpWasPressed()) spindexerPower+=0.01;
            if (gamepad1.dpadDownWasPressed()) spindexerPower-=0.01;
            int detected = colorsensoroperator.getDetected();

            String result;
            if (detected == 1) {
                result = "GREEN";
            } else if (detected == 2) {
                result = "PURPLE";
            } else {
                result = "NONE";
            }

            telemetry.addData("Detected", result);

            float[] hsv = colorsensoroperator.readHSV();
            telemetry.addData("Hue", hsv[0]);
            telemetry.addData("Sat", hsv[1]);
            telemetry.addData("Val", hsv[2]);

            if (result.equals("GREEN")) spindexer.setPower(0);
//            if (colorsensoroperator.getDetected()==1) detections.add("Green");
//            if (colorsensoroperator.getDetected()==2) detections.add("Purple");
//            String tmp="";
//            for (String i : detections){
//                tmp=tmp+i+", ";
//            }
//            telemetry.addLine(tmp);
            telemetry.addData("spindexer Power",spindexerPower);
            telemetry.update();
        }
    }
}