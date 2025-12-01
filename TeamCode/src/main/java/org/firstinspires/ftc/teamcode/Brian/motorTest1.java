package org.firstinspires.ftc.teamcode.Brian;
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

public class motorTest1 extends LinearOpMode{
    DcMotor intake;
    double power=0;
    //main loop
    @Override
    public void runOpMode() {
        intake=hardwareMap.dcMotor.get("intake");
        waitForStart();
        while (opModeIsActive()){
            power-=gamepad1.left_stick_y*0.01;
            intake.setPower(power);
            telemetry.addData("power:",power);
            telemetry.update();
        }
    }
}