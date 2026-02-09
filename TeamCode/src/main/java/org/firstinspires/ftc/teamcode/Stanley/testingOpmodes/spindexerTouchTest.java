package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class spindexerTouchTest extends LinearOpMode {
    TouchSensor input=null;
    CRServo servo=null;
    @Override
    public void runOpMode(){
        input=hardwareMap.get(TouchSensor.class,"touch");
        servo=hardwareMap.get(CRServo.class,"spindexerServo");
        double change=0.1;
        double targetSpeed=0;
        boolean correctingtoggle=false;
        waitForStart();
        while(opModeIsActive()){
            if (gamepad1.yWasPressed()){
                correctingtoggle=!correctingtoggle;
            }
            //shift speed
            if (gamepad1.rightBumperWasPressed()){
                change*=10;
            }else if (gamepad1.leftBumperWasPressed()){
                change/=10;
            }
            telemetry.addData("Change",change);
            if (gamepad1.dpadUpWasPressed()){
                targetSpeed+=change;
            }
            if (gamepad1.dpadDownWasPressed()){
                targetSpeed-=change;
            }
            if (correctingtoggle){
                servo.setPower(targetSpeed);
            }else{
                servo.setPower(0);
            }
            if (input.isPressed()){
                correctingtoggle=false;
                servo.setPower(0);
            }
            telemetry.addData("Holding",correctingtoggle);
            telemetry.addData("Target",targetSpeed);
            telemetry.addData("Pressed",input.isPressed());
            telemetry.update();
        }
    }
}
