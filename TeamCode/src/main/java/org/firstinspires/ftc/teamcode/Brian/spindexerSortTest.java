package org.firstinspires.ftc.teamcode.Brian;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.opModeDataTransfer;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtakeV3FittedAutolaunch;

import java.util.LinkedList;
import java.util.Map;

@TeleOp
public class spindexerSortTest extends OpMode {
    enum SpindexerState {
        STOPPED,
        HOLDING,
        HOLDING_OUTTAKE,
        MANUAL,
        INTAKE,
        OUTTAKE,
        OUTTAKE_SORTED
    }
    // ==================== HARDWARE DECLARATION ====================
    CRServo spindexer = null;
    DcMotor intake=null;

    //Spindexer Encoder
    DcMotorEx spindexerEncoder = null;

    // ==================== Classes ====================
    spindexerColor spindexerOperator=null;

    // ==================== STATE VARIABLES ====================
    SpindexerState spindexerState = SpindexerState.STOPPED;

    // ==================== CONFIGURATION ====================
    static final double SPINDEXER_MANUAL_SPEED=0.6;

    // ==================== WORKING VARIABLES ====================
    int motifindex=0;

    // ==================== TELEMETRY ====================
    FtcDashboard dashboard=FtcDashboard.getInstance();
    Telemetry dashboardtelemetry=dashboard.getTelemetry();

    // ==================== INITIALIZATION ====================
    @Override
    public void init() {
        spindexer = hardwareMap.get(CRServo.class, "spindexerServo");

        // Initialize Encoder
        spindexerEncoder = hardwareMap.get(DcMotorEx.class,"intake");

        intake=hardwareMap.get(DcMotor.class,"intake");

        //Initialize classes
        spindexerOperator=new spindexerColor(spindexer,intake,hardwareMap);

        telemetry.addLine("Robot Initialized and Ready");
        telemetry.update();
    }

    // ==================== MAIN LOOP ====================
    @Override
    public void loop() {
        try {
            updateSpindexerStateMachine();

            // Update telemetry
            updateTelemetry();
        } catch (Exception e) {
            telemetry.addData("ERROR", e.getClass().getSimpleName());
            telemetry.addData("Message", e.getMessage());
            telemetry.update();
            // Skips this loop iteration on exception
        }
    }

    // ==================== SPINDEXER STATE MACHINE ====================
    void updateSpindexerStateMachine(){
        if (gamepad1.aWasPressed()){
            spindexerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindexerEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            gamepad1.rumble(10);
        }
        if (gamepad1.yWasPressed()){
            switch (spindexerState){
                case STOPPED:
                    spindexerState= SpindexerState.INTAKE;
                    spindexerOperator.initSpin();
                    break;
                case HOLDING:
                case OUTTAKE_SORTED:
                case INTAKE:
                    spindexerState= SpindexerState.STOPPED;
                    break;
            }
        }
        if (gamepad1.dpadUpWasPressed()) {
            motifindex += 1;
            motifindex =motifindex% 3;
        }
        if (gamepad1.xWasPressed()){
            spindexerState=SpindexerState.OUTTAKE_SORTED;
            spindexerOperator.initSpin();
        }
        if (gamepad1.left_bumper || gamepad1.right_bumper ||gamepad2.right_bumper){
            spindexerState= SpindexerState.MANUAL;
            if (gamepad1.left_bumper){
                spindexer.setPower(-SPINDEXER_MANUAL_SPEED);
            }
            if (gamepad1.right_bumper || gamepad2.right_bumper){
                spindexer.setPower(SPINDEXER_MANUAL_SPEED);
            }
        }
        if (gamepad1.leftBumperWasReleased() || gamepad1.rightBumperWasReleased() ||gamepad2.rightBumperWasReleased()){
            spindexerState= SpindexerState.STOPPED;
            spindexer.setPower(0);
        }
        if (spindexerState== SpindexerState.INTAKE){
            boolean result=spindexerOperator.spinToIntake();
            if (result){
                spindexerState= SpindexerState.HOLDING;
            }else if (spindexerOperator.detectioncnt==3){
                spindexerState= SpindexerState.STOPPED;
                gamepad1.rumble(100);
            }
        }
        else if (spindexerState== SpindexerState.STOPPED){
            spindexer.setPower(0);
        }
        else if (spindexerState== SpindexerState.HOLDING){
            spindexerOperator.holdSpindexer();
            if (spindexerOperator.intakesensor.isGreen() || spindexerOperator.intakesensor.isPurple()){
                spindexerState= SpindexerState.INTAKE;
                spindexerOperator.initSpin();
            }
        }
        else if (spindexerState== SpindexerState.HOLDING_OUTTAKE){
            spindexerOperator.holdSpindexerOuttake();
            telemetry.addLine("Motif Found!");
            if (gamepad1.rightBumperWasPressed()){
                spindexerState=SpindexerState.STOPPED;
            }
        }
        else if (spindexerState== SpindexerState.OUTTAKE){
            spindexer.setPower(0.7);
        }
        else if (spindexerState== SpindexerState.OUTTAKE_SORTED){
            boolean result=spindexerOperator.spinToMotifV2(motifindex);
            if (result){
                spindexerState= SpindexerState.HOLDING_OUTTAKE;
            }else if (spindexerOperator.detectioncnt==3){
                spindexerState= SpindexerState.STOPPED;
                gamepad1.rumble(100);
            }
        }
    }

    // ==================== TELEMETRY ====================
    void updateTelemetry() {
        telemetry.addLine("=== Spindexer ===");
        telemetry.addData("Motif",spindexerOperator.motifPattern[motifindex]);
        telemetry.addData("Index",motifindex);
        telemetry.addData("State",spindexerState);
        telemetry.addData("Power",spindexerOperator.spindexerPID.power);
        telemetry.addData("Error",spindexerOperator.calculateError(spindexerOperator.outslotsV[spindexerOperator.currentSlot],spindexerOperator.spindexerSensor.getCurrentPosition()));
        telemetry.addData("Target",spindexerOperator.outslotsV[spindexerOperator.currentSlot]);
        telemetry.addData("Encoder Absolute",spindexerOperator.spindexerSensor.getCurrentPosition()%8192);
        telemetry.addData("Encoder",spindexerOperator.spindexerSensor.getCurrentPosition());
        telemetry.addData("Hue",spindexerOperator.intakesensor.readHSV()[0]);
        telemetry.addData("Detected",spindexerOperator.intakesensor.getDetected());

        telemetry.update();
        dashboardtelemetry.update();
    }

    @Override
    public void stop() {
    }
}