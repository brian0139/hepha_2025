package org.firstinspires.ftc.teamcode.Stanley.finalizedOpmodes;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Brian.spindexerColor;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.opModeDataTransfer;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtakeV3;

import java.lang.Math;
import java.util.Map;

@TeleOp
public class teleOpMainNew extends OpMode {

    // ==================== STATE MACHINE ENUMS ====================
    enum FlywheelState {
        IDLE,
        SPINNING,
        STOPPED
    }

    enum TransferState {
        UP,
        STOPPED
    }

    enum IntakeState {
        STOPPED,
        INTAKING,
        AWAITING_SPINDEXER,
        MANUAL
    }

    enum HoodState{
        MANUAL,
        AUTO
    }

    enum TurretState{
        MANUAL,
        AUTO
    }

    // ==================== HARDWARE DECLARATION ====================
    // Drivetrain motors
    DcMotor leftFront = null;
    DcMotor leftBack = null;
    DcMotor rightFront = null;
    DcMotor rightBack = null;

    // Other motors
    DcMotorEx flywheel = null;
    DcMotorEx flywheelR = null;
    DcMotor intake = null;
    DcMotor transfer = null;

    // Servos
    CRServo hoodServo = null;
    CRServo spindexer = null;

    //Analog Input
    AnalogInput spindexerAnalog = null;

    // ==================== Classes ====================
    spindexerColor spindexerOperator=null;
    outtakeV3 outtakeOperator=null;
    MecanumDrive drive=null;

    // ==================== STATE VARIABLES ====================
    FlywheelState flywheelState = FlywheelState.IDLE;
    TransferState transferState = TransferState.STOPPED;
    IntakeState intakeState = IntakeState.STOPPED;
    HoodState hoodState = HoodState.AUTO;
    TurretState turretState = TurretState.AUTO;

    // ==================== CONFIGURATION ====================
    static final double FLYWHEEL_SENSITIVITY = 5;
    static final double FLYWHEEL_EPSILON = 10;
    static final double FLYWHEEL_EXIT_EPSILON = 35;
    static final double FLYWHEEL_DIAMETER = 2.8346456692913386;
    static final double FLYWHEEL_EFFICIENCY = 0.95;
    static final double DRIVE_SPEED = 0.7;
    static final double STRAFE_SPEED = 0.5;
    static final double TWIST_SPEED = 0.5;
    static final double SECONDARY_DILATION = 0.25;

    static final double[] TRANSFER_POWERS = {1, 0};
    static final int TRANSFER_DOWN = 1;
    static final int TRANSFER_UP = 0;

    // ==================== WORKING VARIABLES ====================
    int flywheelSpeed = 2000;
    int targetSpeed = 0;
    boolean atFlywheelTarget=false;
    boolean useInitializer=false;

    // ==================== INITIALIZATION ====================
    @Override
    public void init() {
        // Initialize drivetrain motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Reverse motor directions where needed
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize other motors
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelR = hardwareMap.get(DcMotorEx.class,"flywheelR");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "par1");

        // Initialize servos
        hoodServo = hardwareMap.get(CRServo.class, "hoodServo");
        spindexer = hardwareMap.get(CRServo.class, "spindexerServo");

        // Initialize analog input
        spindexerAnalog = hardwareMap.get(AnalogInput.class,"spindexerAnalog");

        // Set initial positions
        transfer.setPower(TRANSFER_POWERS[TRANSFER_DOWN]);
        transferState = TransferState.STOPPED;

        //Initialize classes
        spindexerOperator=new spindexerColor(spindexer,intake,hardwareMap);
        drive=new MecanumDrive(hardwareMap, opModeDataTransfer.currentPose);
        outtakeOperator=new outtakeV3(hardwareMap,"Red",true,drive);
        outtakeOperator.encoderOffset=opModeDataTransfer.currentHood;
        outtakeOperator.apriltag.init();

        telemetry.addLine("Robot Initialized and Ready");
        telemetry.update();
    }

    // ==================== MAIN LOOP ====================
    @Override
    public void loop() {
        // Update all state machines
        updateFlywheelStateMachine();
        updateTransferStateMachine();
        updateIntakeStateMachine();
        updateDrivetrain();
        updateManual();

        // Update telemetry
        updateTelemetry();
    }

    // ==================== FLYWHEEL STATE MACHINE ====================
    void updateFlywheelStateMachine() {
        // Adjust speed with right stick
        if (Math.abs(gamepad2.right_stick_y) > 0.1) {
            int speedChange = (int)(gamepad2.right_stick_y * FLYWHEEL_SENSITIVITY);
            if (flywheelSpeed - speedChange >= 0) {
                flywheelSpeed -= speedChange;
                if (flywheelState == FlywheelState.SPINNING) {
                    flywheelState = FlywheelState.STOPPED;
                }
            } else {
                flywheelSpeed = 0;
            }
        }

        //Rumble within epsilon
        if (Math.abs(targetSpeed-flywheel.getVelocity())<FLYWHEEL_EPSILON && !atFlywheelTarget){
            atFlywheelTarget=true;
            gamepad2.rumble(100);
        }
        if (Math.abs(targetSpeed-flywheel.getVelocity())>FLYWHEEL_EXIT_EPSILON){
            atFlywheelTarget=false;
        }

        // Toggle flywheel on/off with Y button
        if (gamepad2.yWasPressed()) {
            switch (flywheelState) {
                case STOPPED:

                case IDLE:
                    flywheelState = FlywheelState.SPINNING;
                    targetSpeed = flywheelSpeed;
                    flywheel.setVelocity(targetSpeed);
                    break;
                case SPINNING:
                    flywheelState = FlywheelState.STOPPED;
                    targetSpeed = 0;
                    flywheel.setVelocity(0);
                    break;
            }
        }
    }

    // ==================== TRANSFER STATE MACHINE ====================
    void updateTransferStateMachine() {
        switch (transferState) {
            case STOPPED:
                if (gamepad2.xWasPressed()) {
                    transfer.setPower(TRANSFER_POWERS[TRANSFER_UP]);
                    transferState = TransferState.UP;
                    spindexer.setPower(1);
                    intake.setPower(0.75);
                }
                break;

            case UP:
                if (gamepad2.xWasPressed()) {
                    transfer.setPower(TRANSFER_POWERS[TRANSFER_DOWN]);
                    transferState = TransferState.STOPPED;
                    spindexer.setPower(0);
                    intake.setPower(0);
                }
                break;
        }
    }

    // ==================== INTAKE STATE MACHINE ====================
    void updateIntakeStateMachine() {
        if (gamepad1.right_trigger!=0 || gamepad1.left_trigger!=0){
            intakeState=IntakeState.MANUAL;
            intake.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
        }
        if (gamepad1.right_bumper || gamepad1.left_bumper){
            if (intakeState==IntakeState.AWAITING_SPINDEXER){
                intakeState=IntakeState.INTAKING;
            }
            if (gamepad1.right_bumper) spindexer.setPower(1);
            else if (gamepad1.left_bumper) spindexer.setPower(-1);
        }
        if (gamepad1.rightBumperWasReleased() || gamepad1.leftBumperWasReleased()){
            intakeState=IntakeState.STOPPED;
            spindexer.setPower(0);
        }
        //Toggle intake
        if (gamepad1.yWasPressed()){
            switch (intakeState){
                case MANUAL:
                case STOPPED:
                    intakeState=IntakeState.AWAITING_SPINDEXER;
                    spindexerOperator.detectioncnt=0;
                    break;
                case AWAITING_SPINDEXER:
                case INTAKING:
                    intakeState=IntakeState.STOPPED;
                    spindexer.setPower(0);
                    intake.setPower(0);
                    break;
            }
        }
        if (intakeState==IntakeState.STOPPED){
            intake.setPower(0);
        }
        else if (intakeState==IntakeState.INTAKING){
            intake.setPower(1);
            spindexerOperator.holdSpindexer();
            if (!(spindexerOperator.intakesensor.getDetected()==0)){
                intakeState=IntakeState.AWAITING_SPINDEXER;
                spindexerOperator.detectioncnt=0;
                gamepad1.rumbleBlips(3);
                intake.setPower(0.7);
            }
        }
        else if (intakeState==IntakeState.AWAITING_SPINDEXER){
            boolean result=spindexerOperator.spinToIntake();
            intake.setPower(0.7);
            if (result){
                intakeState=IntakeState.INTAKING;
            }else if (!result && spindexerOperator.detectioncnt==3){
                intakeState=IntakeState.STOPPED;
                gamepad1.rumble(100);
            }
        }
    }

    // ==================== HOOD CONTROL ====================
    void updateHoodControl() {
        hoodServo.setPower(-gamepad2.left_stick_y);
    }

    // ==================== DRIVETRAIN CONTROL ====================
    void updateDrivetrain() {
        // Base control from analog sticks
        double drive = -gamepad1.left_stick_y * DRIVE_SPEED;
        double strafe = -gamepad1.left_stick_x * STRAFE_SPEED;
        double twist = -gamepad1.right_stick_x * TWIST_SPEED;

        // D-pad overrides for precise movement
        if (gamepad1.dpad_up) drive = DRIVE_SPEED;
        if (gamepad1.dpad_down) drive = -DRIVE_SPEED;
        if (gamepad1.dpad_left) strafe = STRAFE_SPEED;
        if (gamepad1.dpad_right) strafe = -STRAFE_SPEED;

        // Face buttons for slow precise movement
        if (gamepad1.y) drive = DRIVE_SPEED * SECONDARY_DILATION;
        if (gamepad1.a) drive = -DRIVE_SPEED * SECONDARY_DILATION;
        if (gamepad1.x) strafe = STRAFE_SPEED * SECONDARY_DILATION;
        if (gamepad1.b) strafe = -STRAFE_SPEED * SECONDARY_DILATION;

        // Calculate mecanum wheel speeds
        double[] speeds = {
                (drive - strafe - twist), // leftFront
                (drive + strafe + twist), // rightFront
                (drive + strafe - twist), // leftBack
                (drive - strafe + twist)  // rightBack
        };

        // Normalize speeds if any exceed 1.0
        double max = Math.abs(speeds[0]);
        for (int i = 1; i < speeds.length; i++) {
            if (Math.abs(speeds[i]) > max) {
                max = Math.abs(speeds[i]);
            }
        }

        if (max > 1.0) {
            for (int i = 0; i < speeds.length; i++) {
                speeds[i] /= max;
            }
        }

        // Apply speeds to motors
        leftFront.setPower(speeds[0]);
        rightFront.setPower(speeds[1]);
        leftBack.setPower(speeds[2]);
        rightBack.setPower(speeds[3]);
    }

    // ==================== TELEMETRY ====================
    void updateTelemetry() {
        telemetry.addLine("=== Flywheel ===");
        telemetry.addData("Target Speed",flywheelSpeed);
        telemetry.addData("Actual Speed",flywheel.getVelocity());
        telemetry.addLine("=== Toggles ===");
        telemetry.addData("Auto Hood",hoodState==HoodState.AUTO);
        telemetry.addData("Auto Turret",turretState==TurretState.AUTO);
        telemetry.addLine("=== Limelight ===");
        telemetry.addData("Has Target",outtakeOperator.apriltag.hasValidTarget());
        telemetry.addLine("=== Intake ===");
        if (intakeState==IntakeState.AWAITING_SPINDEXER){
            telemetry.addData("Intake State","Awaiting Spindexer");
        }else if (intakeState==IntakeState.INTAKING){
            telemetry.addData("Intake State","Intake");
        }else if (intakeState==IntakeState.MANUAL){
            telemetry.addData("Intake State","Manual");
        }else if (intakeState==IntakeState.STOPPED){
            telemetry.addData("Intake State","Stopped");
        }

        telemetry.update();
    }

    // ==================== Manual Override/Misc ====================
    void updateManual(){
        if (gamepad2.leftBumperWasPressed()){
            outtakeOperator.initHoodAngleBlocking();
            gamepad2.rumbleBlips(2);
            gamepad2.rumble(50);
        }
        if (gamepad2.dpadUpWasPressed()){
            switch (hoodState){
                case MANUAL:
                    hoodState=HoodState.AUTO;
                    break;
                case AUTO:
                    hoodState=HoodState.MANUAL;
                    break;
            }
        }
        if (gamepad2.dpadDownWasPressed()){
            switch (turretState){
                case MANUAL:
                    turretState=TurretState.AUTO;
                    outtakeOperator.turnPID.init();
                    break;
                case AUTO:
                    turretState=TurretState.MANUAL;
                    break;
            }
        }
        Map<String,String> output=outtakeOperator.findOptimalLaunch(outtakeOperator.getDistance(),40,40.03,66.81,outtakeOperator.calculateCurvedExitSpeed(2100,FLYWHEEL_DIAMETER,FLYWHEEL_EFFICIENCY),90,170,160,386.4,10,0.1,100);
        if (hoodState==HoodState.AUTO && outtakeOperator.apriltag.hasValidTarget()){
            outtakeOperator.setHood(Double.parseDouble(output.get("angle")));
            flywheelSpeed=(int) Math.round(outtakeOperator.calculateRequiredRPM(Double.parseDouble(output.get("velocity")),FLYWHEEL_DIAMETER,FLYWHEEL_EFFICIENCY));
        }else{
            updateHoodControl();
        }
        if (turretState==TurretState.AUTO){
            outtakeOperator.setPipeLine(5);
            outtakeOperator.autoturn();
        }else{
            outtakeOperator.turretServo.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
        }
    }

    @Override
    public void stop() {
        // Clean shutdown
        flywheel.setVelocity(0);
        intake.setPower(0);
        hoodServo.setPower(0);
    }
}