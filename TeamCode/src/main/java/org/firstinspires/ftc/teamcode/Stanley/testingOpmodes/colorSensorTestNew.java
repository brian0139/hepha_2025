package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Alvin.colorSensor;
import java.util.Vector;

@TeleOp

public class colorSensorTestNew extends LinearOpMode{
    CRServo spindexer;
    colorSensor colorsensoroperatorOuttake;
    colorSensor colorSensorIntake;
    ElapsedTime timer=new ElapsedTime();

    //main loop
    @Override
    public void runOpMode() {
        //initiate drivetrain motors
        spindexer=hardwareMap.get(CRServo.class,"spindexerServo");
        colorsensoroperatorOuttake=new colorSensor(hardwareMap,"outtakeSensor");
        colorSensorIntake=new colorSensor(hardwareMap,"intakeSensor");
        double spindexerPower=0.75;
        int waittime=400;
        boolean waiting=false;
        //telemetry message to signify robot waiting
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        Vector<String> detections=new Vector<>();
        //wait for driver to press play
        waitForStart();
        //repeat until opmode ends
        while (opModeIsActive()){
            if (gamepad1.left_bumper){
                spindexer.setPower(-spindexerPower);
            }else if (gamepad1.right_bumper){
                spindexer.setPower(spindexerPower);
            }else{
                spindexer.setPower(0);
            }

            int detected = colorsensoroperatorOuttake.getDetected();
            int detectedIntake=colorSensorIntake.getDetected();

            String result;
            if (detected == 1) {
                result = "GREEN";
            } else if (detected == 2) {
                result = "PURPLE";
            } else {
                result = "NONE";
            }
            String resultintake;
            if (detectedIntake == 1) {
                resultintake = "GREEN";
            } else if (detectedIntake == 2) {
                resultintake = "PURPLE";
            } else {
                resultintake = "NONE";
            }



            telemetry.addData("DetectedOuttake", result);

            float[] hsv = colorsensoroperatorOuttake.readHSV();
            telemetry.addData("HueOuttake", hsv[0]);
            telemetry.addData("SatOuttake", hsv[1]);
            telemetry.addData("ValOuttake", hsv[2]);

            telemetry.addData("DetectedIntake", resultintake);

            float[] hsv_ = colorSensorIntake.readHSV();
            telemetry.addData("HueIntake", hsv_[0]);
            telemetry.addData("SatIntake", hsv_[1]);
            telemetry.addData("ValIntake", hsv_[2]);

//            if (result.equals("GREEN")) {
//                spindexer.setPower(-0.01);
//                waiting=true;
//                timer.reset();
//            }
//            if (waiting && timer.milliseconds()>=waittime){
//                waiting=false;
//                spindexer.setPower(0);
//            }

//            if (colorsensoroperator.getDetected()==1) detections.add("Green");
//            if (colorsensoroperator.getDetected()==2) detections.add("Purple");
//            String tmp="";
//            for (String i : detections){
//                tmp=tmp+i+", ";
//            }
//            telemetry.addLine(tmp);
            telemetry.addData("spindexer Power",spindexerPower);
            telemetry.addData("Servo Power",spindexer.getPower());
            telemetry.addData("Waittime",waittime);
            telemetry.update();
        }
    }
}