package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class encoderTest extends LinearOpMode {
    Encoder par0, par1, perp;
    DcMotorEx encoder=null;
    CRServo servo=null;
    @Override
    public void runOpMode() throws InterruptedException{
//        encoder=hardwareMap.get(DcMotorEx.class,"leftFront");
//        encoder.setDirection(DcMotorSimple.Direction.REVERSE);
//        servo=hardwareMap.get(CRServo.class,"hoodServo");
        //encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        while (opModeIsActive()){
//            if (gamepad1.yWasPressed()) {
//                encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
////                while (encoder.getCurrentPosition() > -3 * 8192) {
////                    servo.setPower(1);
////                    telemetry.addData("Encoder", encoder.getCurrentPosition());
////                    telemetry.addData("target", -3 * 8192);
////                    telemetry.update();
////                }
////                servo.setPower(0);
//            }
            par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightBack")));
            par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "par1")));
            perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFront")));

            // TODO: reverse encoder directions if needed
            par0.setDirection(DcMotorSimple.Direction.REVERSE);
            par1.setDirection(DcMotorSimple.Direction.REVERSE);
            perp.setDirection(DcMotorSimple.Direction.REVERSE);
//            encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("Par0",par0.getPositionAndVelocity().position);
            telemetry.addData("Par1",par1.getPositionAndVelocity().position);
            telemetry.addData("Perp",perp.getPositionAndVelocity().position);
            telemetry.update();
//            servo.setPower(1);
//            telemetry.addData("Encoder",encoder.getCurrentPosition());
//            telemetry.update();
        }
    }
}
