package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class encoderTest extends LinearOpMode {

    DcMotorEx encoder=null;
    CRServo servo=null;
    @Override
    public void runOpMode() throws InterruptedException{
        encoder=hardwareMap.get(DcMotorEx.class,"leftBack");
        servo=hardwareMap.get(CRServo.class,"hoodServo");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        while (opModeIsActive()){
            servo.setPower(1);
            telemetry.addData("Encoder",encoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
