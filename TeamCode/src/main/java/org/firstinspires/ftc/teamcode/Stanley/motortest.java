package org.firstinspires.ftc.teamcode.Stanley;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Math;
@TeleOp

public class motortest extends LinearOpMode{
    DcMotor tmpmotor;
    DcMotor tmpmotor2;
    double power=0;
    //main loop
    @Override
    public void runOpMode() {
        tmpmotor=hardwareMap.dcMotor.get("tmpmotortest");
        tmpmotor2=hardwareMap.dcMotor.get("tmpmotor");
        waitForStart();
        while (opModeIsActive()){
            power-=gamepad1.left_stick_y*0.01;
            tmpmotor.setPower(power);
            tmpmotor2.setPower(power);
            telemetry.addData("power:",power);
            telemetry.update();
        }
    }
}