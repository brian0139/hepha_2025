package org.firstinspires.ftc.teamcode.Brian;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Alvin.colorSensor;
import java.util.Vector;
import java.util.*;

@TeleOp

public class spindexerColorTest extends LinearOpMode{
    public static double FLYWHEEL_TARGET_TPS = 1531;
    public static double FLYWHEEL_TOLERANCE_TPS = 50;
    public static double TRANSFER_UP_POWER = -1.0;
    public static double TRANSFER_DOWN_POWER = 0.0;

    CRServo spindexer;
    colorSensor colorsensoroperator;
    spindexerColor spindexercolor;
    DcMotorEx spindexerAnalog;
    DcMotor intake;
    DcMotor transfer;
    DcMotorEx flywheel;
    DcMotorEx flywheelR;
    int motifIndex=0;

    //main loop
    @Override
    public void runOpMode() {
        //initiate drivetrain motors
        spindexer=hardwareMap.get(CRServo.class,"spindexerServo");
        colorsensoroperator=new colorSensor(hardwareMap,"outtakeSensor");
        spindexerAnalog=hardwareMap.get(DcMotorEx.class,"spindexerAnalog");
        intake=hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "par1");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheelR = hardwareMap.get(DcMotorEx.class, "flywheelR");
        flywheelR.setDirection(DcMotorSimple.Direction.REVERSE);


        spindexercolor=new spindexerColor(spindexer,intake,hardwareMap);

        boolean tmp=false;
        boolean spintointaketoggle=false;
//        FtcDashboard dashboard=FtcDashboard.getInstance();
//        telemetry= dashboard.getTelemetry();
        //telemetry message to signify robot waiting
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        Vector<String> detections=new Vector<>();
        //wait for driver to press play
        waitForStart();
        //repeat until opmode ends
        while (opModeIsActive()) {
            if (gamepad1.yWasPressed()) {
                motifIndex++;
                motifIndex %= 3;
            }
            if (gamepad1.xWasPressed()){
                spintointaketoggle=!spintointaketoggle;
            }
            if (spintointaketoggle){
                // Run sorter, flywheel, and transfer together while motif sorting is active.
                spindexercolor.spinToMotif(motifIndex);
                runFlywheelAtTarget(FLYWHEEL_TARGET_TPS, FLYWHEEL_TOLERANCE_TPS);
                transfer.setPower(TRANSFER_UP_POWER);
            }else{
                spindexercolor.spindexerServo.setPower(0);
                flywheel.setPower(0);
                flywheelR.setPower(0);
                transfer.setPower(TRANSFER_DOWN_POWER);
            }
            telemetry.addData("spindexer slots", Arrays.toString(spindexercolor.motifPattern));
            telemetry.addData("current motif index", motifIndex);
            telemetry.addData("current motif color", spindexercolor.motifPattern[motifIndex]);
            int detected = colorsensoroperator.getDetected();
//
            String result;
            if (detected == 1) {
                result = "GREEN";
            } else if (detected == 2) {
                result = "PURPLE";
            } else {
                result = "NONE";
            }

            telemetry.addData("Detected", result);


            telemetry.addData("voltage", spindexerAnalog.getCurrentPosition());
            telemetry.addData("Power",spindexercolor.spindexerPID.power);
            telemetry.addData("target voltage",spindexercolor.inslotsV[spindexercolor.currentSlot]);
            telemetry.addData("triggered",!((spindexercolor.spindexerSensor.getCurrentPosition()>=spindexercolor.inslotsV[spindexercolor.currentSlot]-0.05)&&(spindexercolor.spindexerSensor.getCurrentPosition()<=spindexercolor.inslotsV[spindexercolor.currentSlot]+0.05)));
            telemetry.addData("detectioncnt",spindexercolor.detectioncnt);
            telemetry.addData("current slot",spindexercolor.currentSlot);
            telemetry.update();
        }

        spindexercolor.spindexerServo.setPower(0);
        flywheel.setPower(0);
        flywheelR.setPower(0);
        transfer.setPower(TRANSFER_DOWN_POWER);
    }

    private void runFlywheelAtTarget(double targetTicksPerSec, double toleranceTicksPerSec) {
        double avgVelocity = (Math.abs(flywheel.getVelocity()) + Math.abs(flywheelR.getVelocity())) / 2.0;
        if (avgVelocity < targetTicksPerSec - toleranceTicksPerSec) {
            flywheel.setPower(1.0);
            flywheelR.setPower(1.0);
        } else if (avgVelocity > targetTicksPerSec + toleranceTicksPerSec) {
            flywheel.setPower(0.0);
            flywheelR.setPower(0.0);
        }
    }
}
