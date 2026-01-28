package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class encoderTest extends LinearOpMode {

    DcMotorEx encoder=null;
    CRServo servo=null;
    @Override
    public void runOpMode() throws InterruptedException{
        encoder=hardwareMap.get(DcMotorEx.class,"leftBack");
        encoder.setDirection(DcMotorSimple.Direction.REVERSE);
        servo=hardwareMap.get(CRServo.class,"hoodServo");
        //encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.yWasPressed()) {
                encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                while (encoder.getCurrentPosition() > -3 * 8192) {
                    servo.setPower(1);
                    telemetry.addData("Encoder", encoder.getCurrentPosition());
                    telemetry.addData("target", -3 * 8192);
                    telemetry.update();
                }
                servo.setPower(0);
            }
//            encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("Encoder",encoder.getCurrentPosition());
            telemetry.update();
//            servo.setPower(1);
//            telemetry.addData("Encoder",encoder.getCurrentPosition());
//            telemetry.update();
        }
    }
}
