package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtake;

@TeleOp
public class limelightautoturntest extends LinearOpMode {

    private Limelight3A limelight;
    private outtake outtakeOperator;    // drivetrain wheel motor declaration
    private DcMotor leftFront=null;
    private DcMotor leftBack=null;
    private DcMotor rightFront=null;
    private DcMotor rightBack=null;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //initiate drivetrain motors
        leftFront   = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack    = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront   = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack   = hardwareMap.get(DcMotor.class, "rightBack");
        //Reverse motor directions where needed
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        boolean aimtoggle=false;
        outtakeOperator=new outtake(hardwareMap,null,"Red",leftFront,rightFront,leftBack,rightBack,null,null,true);

        waitForStart();
        while (opModeIsActive()) {
            outtakeOperator.TURN_GAIN-= gamepad1.left_stick_x*0.000001;
            if (outtakeOperator.TURN_GAIN<0) outtakeOperator.TURN_GAIN=0;
            if (gamepad1.dpadDownWasPressed()){
                outtakeOperator.TURN_GAIN-=0.001;
            }
            if (gamepad1.dpadUpWasPressed()){
                outtakeOperator.TURN_GAIN+=0.001;
            }
            if (gamepad1.yWasPressed()) aimtoggle=!aimtoggle;
            if (aimtoggle) outtakeOperator.autoturn();
            telemetry.addData("Turn Gain",outtakeOperator.TURN_GAIN);
            telemetry.addData("Autoaiming",aimtoggle);
            telemetry.update();
        }
    }
}