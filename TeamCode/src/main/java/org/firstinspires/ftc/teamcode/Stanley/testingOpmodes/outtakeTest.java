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

public class outtakeTest extends LinearOpMode{
    private CRServo hoodServo=null;

    //main loop
    @Override
    public void runOpMode() {
        double servoRPM=50;
        int msPerRotation=1/50*60*1000;
        double hoodAngle=30;
        //outtake instance
        hoodServo=hardwareMap.get(CRServo.class,"hoodServo");
        outtake outtakeOperator = new outtake(hardwareMap, null, "Red",null,null,null,null,hoodServo,null,false);
        outtakeOperator.hoodAngle=hoodAngle;
        outtakeOperator.savehoodAngle=hoodAngle;

        //telemetry message to signify robot waiting
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        //wait for driver to press play
        waitForStart();
        while (opModeIsActive()){
            hoodAngle+=gamepad1.left_stick_y*0.15;
            hoodServo.setPower(-gamepad1.right_stick_x);
            if (gamepad1.rightBumperWasPressed()){
                hoodServo.setPower(1);
                sleep(msPerRotation);
                hoodServo.setPower(0);
            }
            if (gamepad1.leftBumperWasPressed()){
                hoodServo.setPower(-1);
                sleep(msPerRotation);
                hoodServo.setPower(0);
            }
            if (gamepad1.yWasPressed()){
                outtakeOperator.setHood(hoodAngle,true);
            }
        }
    }
}