package org.firstinspires.ftc.teamcode.Stanley.finalizedOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Brian.spindexerColor;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.opModeDataTransfer;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtakeV3FittedAutolaunch;

import java.util.LinkedList;
import java.util.Map;

@TeleOp
public class teleOpMainNewRed extends OpMode {

    // ==================== STATE MACHINE ENUMS ====================
    enum FlywheelState {
        IDLE,
        SPINNING,
        STOPPED
    }

    enum TransferState {
        UP,
        STOPPED,
        DOWN
    }

    enum SpindexerState {
        STOPPED,
        HOLDING,
        MANUAL,
        INTAKE,
        OUTTAKE,
        OUTTAKE_SORTED
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

    //Spindexer Encoder
    DcMotorEx spindexerEncoder = null;

    //Touch Sensor
    TouchSensor spindexerTouch = null;

    //Indicator Light
    Servo flywheelIndicator=null;

    // ==================== Classes ====================
    spindexerColor spindexerOperator=null;
    outtakeV3FittedAutolaunch outtakeOperator=null;
    MecanumDrive driveTrain=null;

    // ==================== STATE VARIABLES ====================
    FlywheelState flywheelState = FlywheelState.IDLE;
    TransferState transferState = TransferState.STOPPED;
    SpindexerState spindexerState = SpindexerState.STOPPED;
    HoodState hoodState = HoodState.MANUAL;
    TurretState turretState = TurretState.MANUAL;

    // ==================== CONFIGURATION ====================
    static final double FLYWHEEL_SENSITIVITY = 5;
    static final double FLYWHEEL_EPSILON = 45;
    static final double FLYWHEEL_EXIT_EPSILON = 65;
    static final double SPINDEXER_MANUAL_SPEED=0.75;
    static final double DRIVE_SPEED = 0.7;
    static final double STRAFE_SPEED = 1;
    static final double TWIST_SPEED = 0.5;
    static final double FLYWHEEL_IDLE_SPEED = 600;
    static final int MIN_SAMPLE_SIZE=3;
    static final int MAX_SAMPLE_SIZE=20;
    static final double FLYWHEEL_INDICATOR_COLOR=0.5;

    static final double[] TRANSFER_POWERS = {-1, 0};
    static final int TRANSFER_DOWN = 1;
    static final int TRANSFER_UP = 0;

    // ==================== WORKING VARIABLES ====================
    double flywheelSpeed = 2000;
    double targetSpeed = 0;
    boolean atFlywheelTarget=false;
    //Spindexer Touch Sensor Edge Detection
    boolean lastPressed=false;

    //LL sample queue
    LinkedList<Double> LLsample=new LinkedList<>();

    ElapsedTime timer=new ElapsedTime();
    ElapsedTime LLsampleTimer=new ElapsedTime();
    ElapsedTime transferTimer =new ElapsedTime();
    Map<String, String> output = Map.of(
            "angle", "0",
            "velocity","0"
    );

    // ==================== TELEMETRY ====================
    FtcDashboard dashboard=FtcDashboard.getInstance();
    Telemetry dashboardtelemetry=dashboard.getTelemetry();

    // ==================== TESTING ====================
    double minVoltage=20.0;
    VoltageSensor batteryVoltageSensor = null;
//    int makeBallCnt=-1;
//    int ballcnt=0;
//    double[] launchVelocities=new double[]{0,0,0};
//    double previousRateofChange=0;
//    double previousSpeed=0;
//    ElapsedTime flywheelDeltaTimer=new ElapsedTime();
//    File file= AppUtil.getInstance().getSettingsFile("./sdcard/FIRST/shootingData.csv");
//    //Header
//    StringBuilder data = new StringBuilder("Make Ball Cnt,Hood Encoder,Flywheel Speed, Distance, Real Max Flywheel Speed\n");

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
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(9.7,0.7,1.3,3.8));
        flywheelR = hardwareMap.get(DcMotorEx.class,"flywheelR");
        flywheelR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelR.setDirection(DcMotorSimple.Direction.FORWARD);
//        flywheelR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(9.7,0.7,1.3,3.8));
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "par1");

        // Initialize servos
        hoodServo = hardwareMap.get(CRServo.class, "hoodServo");
        spindexer = hardwareMap.get(CRServo.class, "spindexerServo");

        //Initialize Lights
        flywheelIndicator = hardwareMap.get(Servo.class,"flywheelIndicator");

        // Initialize Encoder
        spindexerEncoder = hardwareMap.get(DcMotorEx.class,"intake");

        // Initialize touch sensor
        spindexerTouch = hardwareMap.get(TouchSensor.class,"touch");

        // Set initial positions
        transfer.setPower(TRANSFER_POWERS[TRANSFER_DOWN]);
        transferState = TransferState.STOPPED;

        //Initialize classes
        spindexerOperator=new spindexerColor(spindexer,intake,hardwareMap);
        driveTrain=new MecanumDrive(hardwareMap, opModeDataTransfer.currentPose);
        outtakeOperator=new outtakeV3FittedAutolaunch(hardwareMap,"Red",true,driveTrain);
        //TODO:delete after testing voltage
        batteryVoltageSensor = hardwareMap.get(VoltageSensor .class, "Control Hub");
        outtakeOperator.setPipeLine(0);
        outtakeOperator.apriltag.init();

        telemetry.addLine("Robot Initialized and Ready");
        telemetry.update();
    }

    // ==================== MAIN LOOP ====================
    @Override
    public void loop() {
        try {
            // Update all state machines
            updateFlywheelStateMachine();
            updateTransferStateMachine();
            updateIntakeStateMachine();
            updateSpindexerStateMachine();
            updateDrivetrain();
            updateManual();

            // Update telemetry
            updateTelemetry();
        } catch (Exception e) {
            telemetry.addData("ERROR", e.getClass().getSimpleName());
            telemetry.addData("Message", e.getMessage());
            telemetry.update();
            // Skips this loop iteration on exception
        }
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
        if (Math.abs(targetSpeed+flywheel.getVelocity())<FLYWHEEL_EPSILON && !atFlywheelTarget){
            atFlywheelTarget=true;
            gamepad2.rumble(100);
            flywheelIndicator.setPosition(FLYWHEEL_INDICATOR_COLOR);
        }
        if (Math.abs(targetSpeed+flywheel.getVelocity())>FLYWHEEL_EXIT_EPSILON){
            atFlywheelTarget=false;
            flywheelIndicator.setPosition(0);
        }

        // Toggle flywheel on/off with Y button
        if (gamepad2.yWasPressed()) {
            switch (flywheelState) {
                case STOPPED:
                case IDLE:
                    flywheelState = FlywheelState.SPINNING;
                    targetSpeed = flywheelSpeed;
                    outtakeOperator.spin_flywheel(targetSpeed,50);
                    break;
                case SPINNING:
                    flywheelState = FlywheelState.STOPPED;
                    targetSpeed = FLYWHEEL_IDLE_SPEED;
                    outtakeOperator.spin_flywheel(targetSpeed,50);
//                    //TODO:Testing
//                    ballcnt=0;
//                    previousRateofChange=0;
                    break;
            }
        }
        if (flywheelState == FlywheelState.STOPPED){
            targetSpeed = FLYWHEEL_IDLE_SPEED;
            outtakeOperator.spin_flywheel(FLYWHEEL_IDLE_SPEED,50);
        }
//        //TODO:Testing
//        if (gamepad2.dpadLeftWasPressed()){
//            launchVelocities=new double[]{0,0,0};
//        }
//        if (flywheelState!=FlywheelState.STOPPED){
////            maxFlywheelSpeed=Math.max(maxFlywheelSpeed,flywheelR.getVelocity());
//            double rateofchange=(flywheelR.getVelocity()-previousSpeed)/flywheelDeltaTimer.milliseconds();
//            if (rateofchange<0 && previousRateofChange>0){
//                launchVelocities[ballcnt%3]=previousSpeed;
//                ballcnt++;
//            }
//            previousSpeed=flywheelR.getVelocity();
//            previousRateofChange=rateofchange;
//        }
    }

    // ==================== TRANSFER STATE MACHINE ====================
    void updateTransferStateMachine() {
        switch (transferState) {
            case STOPPED:
                if (gamepad2.xWasPressed()) {
                    transfer.setPower(TRANSFER_POWERS[TRANSFER_UP]);
                    transferState = TransferState.UP;
                    spindexerState=SpindexerState.OUTTAKE;
                    intake.setPower(0.75);
                }
                break;

            case UP:
                if (gamepad2.xWasPressed()) {
                    transfer.setPower(1);
                    transferState = TransferState.DOWN;
                    transferTimer.reset();
                    spindexerState=SpindexerState.STOPPED;
                    intake.setPower(0);
                }
                break;
            case DOWN:
                if (transferTimer.milliseconds()>=500){
                    transfer.setPower(0);
                    transferState=TransferState.STOPPED;
                    break;
                }
        }
    }

    // ==================== INTAKE STATE MACHINE ====================
    void updateIntakeStateMachine() {
        intake.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
    }

    // ==================== SPINDEXER STATE MACHINE ====================
    void updateSpindexerStateMachine(){
        //Reset spindexer encoder on touch sensor press
        if (spindexerTouch.isPressed() && !lastPressed){
            spindexerOperator.resetSpindexerTouch();
            lastPressed=true;
        }else{
            lastPressed=spindexerTouch.isPressed();
        }
        if (gamepad1.aWasPressed()){
            spindexerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindexerEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            spindexerOperator.encoderOffset=0;
        }
        if (gamepad1.yWasPressed()){
            switch (spindexerState){
                case STOPPED:
                    spindexerState=SpindexerState.INTAKE;
                    spindexerOperator.initSpin();
                    break;
                case HOLDING:
                case INTAKE:
                    spindexerState=SpindexerState.STOPPED;
                    break;
            }
        }
        if (gamepad1.left_bumper || gamepad1.right_bumper ||gamepad2.right_bumper){
            spindexerState=SpindexerState.MANUAL;
            if (gamepad1.left_bumper){
                spindexer.setPower(-SPINDEXER_MANUAL_SPEED);
            }
            if (gamepad1.right_bumper || gamepad2.right_bumper){
                spindexer.setPower(SPINDEXER_MANUAL_SPEED);
            }
        }
        if (gamepad1.leftBumperWasReleased() || gamepad1.rightBumperWasReleased() ||gamepad2.rightBumperWasReleased()){
            spindexerState=SpindexerState.STOPPED;
            spindexer.setPower(0);
        }
        if (spindexerState==SpindexerState.INTAKE){
            boolean result=spindexerOperator.spinToIntake();
            if (result){
                spindexerState= SpindexerState.HOLDING;
            }else if (spindexerOperator.detectioncnt==3){
                spindexerState= SpindexerState.STOPPED;
                gamepad1.rumble(100);
            }
        }
        else if (spindexerState==SpindexerState.STOPPED){
            spindexer.setPower(0);
        }
        else if (spindexerState==SpindexerState.HOLDING){
            spindexerOperator.holdSpindexer();
            if (spindexerOperator.intakesensor.isGreen() || spindexerOperator.intakesensor.isPurple()){
                spindexerState=SpindexerState.INTAKE;
                spindexerOperator.initSpin();
            }
        }
        else if (spindexerState==SpindexerState.OUTTAKE){
            spindexer.setPower(0.7);
        }
        else if (spindexerState==SpindexerState.OUTTAKE_SORTED){
            spindexerOperator.spinToMotif(1);
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

        double[] speeds = {
                (drive - strafe - twist), // leftFront
                (drive + strafe + twist), // rightFront
                (drive + strafe - twist), // leftBack
                (drive - strafe + twist)  // rightBack
        };

        double max = Math.abs(speeds[0]);
        for (int i = 0; i < speeds.length; i++) {
            if (Math.abs(speeds[i]) > max) {
                max = Math.abs(speeds[i]);
            }
        }

        if (max > 1.0) {
            for (int i = 0; i < speeds.length; i++) {
                speeds[i] /= max;
            }
        }

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
        dashboardtelemetry.addData("Target Speed",flywheelSpeed);
        dashboardtelemetry.addData("Actual Speed",flywheel.getVelocity());
        telemetry.addLine("=== Toggles ===");
        telemetry.addData("Auto Hood",hoodState==HoodState.AUTO);
        telemetry.addData("Auto Turret",turretState==TurretState.AUTO);
        telemetry.addLine("=== Spindexer ===");
        telemetry.addData("State",spindexerState);
        telemetry.addData("Hue",spindexerOperator.intakesensor.readHSV()[0]);
        telemetry.addData("Detected",spindexerOperator.intakesensor.getDetected());
        telemetry.addData("Spindexer Absolute Position",(spindexerOperator.spindexerSensor.getCurrentPosition()+spindexerOperator.encoderOffset)%8192);
        telemetry.addLine("=== Limelight ===");
        telemetry.addData("Has Target",outtakeOperator.apriltag.hasValidTarget());
        telemetry.addData("Offset(Deg)",outtakeOperator.apriltag.getYaw());
        telemetry.addData("Power",outtakeOperator.turnPID.power);
        telemetry.addData("Area",outtakeOperator.getDistance());
//        dashboardtelemetry.addData("Has Target",outtakeOperator.apriltag.hasValidTarget());
//        dashboardtelemetry.addData("Offset(Deg)",outtakeOperator.apriltag.getYaw());
//        dashboardtelemetry.addData("Power",outtakeOperator.turnPID.power);
//        dashboardtelemetry.addData("Distance",outtakeOperator.getDistance());
        telemetry.addLine("=== Drivetrain ===");
        telemetry.addData("Left Front",leftFront.getPower());
        telemetry.addData("Right Front",rightFront.getPower());
        telemetry.addData("Left Back",leftBack.getPower());
        telemetry.addData("Right Back",rightBack.getPower());
        telemetry.addData("Drive",-gamepad1.left_stick_y * DRIVE_SPEED);
        telemetry.addData("Strafe",-gamepad1.left_stick_x * STRAFE_SPEED);
        telemetry.addData("Twist",-gamepad1.right_stick_x * TWIST_SPEED);
        dashboardtelemetry.addLine("=== Drivetrain ===");
        dashboardtelemetry.addData("Left Front",leftFront.getPower());
        dashboardtelemetry.addData("Right Front",rightFront.getPower());
        dashboardtelemetry.addData("Left Back",leftBack.getPower());
        dashboardtelemetry.addData("Right Back",rightBack.getPower());
        telemetry.addLine("=== LOG ===");
        telemetry.addData("Hood Encoder(LOG)",outtakeOperator.hoodEncoder.getCurrentPosition());
        telemetry.addData("Flywheel Target Speed(LOG)",flywheelSpeed);
        telemetry.addData("Area(LOG)",outtakeOperator.getDistance());
//        for (int i=0;i<3;i++){
//            telemetry.addData("Ball "+(i+1),launchVelocities[i]);
//        }
//        telemetry.addData("Max Flywheel Speed(LOG)",maxFlywheelSpeed);
        dashboardtelemetry.addData("Hood Encoder(LOG)",outtakeOperator.hoodEncoder.getCurrentPosition());
        dashboardtelemetry.addData("Flywheel Target Speed(LOG)",flywheelSpeed);
        dashboardtelemetry.addData("Area(LOG)",outtakeOperator.getDistance());
        minVoltage=Math.min(minVoltage,batteryVoltageSensor.getVoltage());
        telemetry.addLine("=== Voltage ===");
        telemetry.addData("MinVoltage",minVoltage);
        dashboardtelemetry.addLine("=== Voltage ===");
        dashboardtelemetry.addData("MinVoltage",minVoltage);
//        dashboardtelemetry.addData("Max Flywheel Speed(LOG)",maxFlywheelSpeed);
//        for (int i=0;i<3;i++){
//            dashboardtelemetry.addData("Ball "+(i+1),launchVelocities[i]);
//        }

        telemetry.update();
        dashboardtelemetry.update();
    }

    // ==================== Manual Override/Misc ====================
    void updateManual(){
        if (gamepad2.leftBumperWasPressed()){
            outtakeOperator.resetHoodAngle();
            gamepad2.rumbleBlips(2);
            gamepad2.rumble(50);
        }
        if (gamepad2.dpadDownWasPressed()){
            switch (hoodState){
                case MANUAL:
                    hoodState=HoodState.AUTO;
                    outtakeOperator.hoodPID.init();
                    break;
                case AUTO:
                    hoodState=HoodState.MANUAL;
                    outtakeOperator.hoodServo.setPower(0);
                    break;
            }
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
//        if (gamepad2.rightStickButtonWasPressed()){
//            data.append(makeBallCnt).append(",")
//                    .append(outtakeOperator.hoodEncoder.getCurrentPosition()).append(",")
//                    .append(flywheelSpeed).append(",")
//                    .append(outtakeOperator.getDistance()).append(",")
//                    .append(maxFlywheelSpeed).append("\n");
//            dashboardtelemetry.addData("Make Ball Cnt(LOG)",makeBallCnt);
//        }
//        else if (gamepad2.dpadLeftWasPressed()){
//            makeBallCnt--;
//        }
//        else if (gamepad2.dpadRightWasPressed()){
//            makeBallCnt++;
//        }
//        telemetry.addData("Make Ball Count",makeBallCnt);
//        dashboardtelemetry.addData("Make Ball Count",makeBallCnt);

//        telemetry.addData("Angle Target",Double.parseDouble(output.get("angle")));
//        telemetry.addData("Encoder Reading",outtakeOperator.hoodEncoder.getCurrentPosition());
//        telemetry.addData("Encoder Target",(66.81-Double.parseDouble(output.get("angle")))/outtakeOperator.servoDegPerRot*outtakeOperator.ticksPerRevHood);
//        telemetry.addData("Error",((66.81-Double.parseDouble(output.get("angle")))/outtakeOperator.servoDegPerRot*outtakeOperator.ticksPerRevHood)-outtakeOperator.hoodEncoder.getCurrentPosition());
//        telemetry.addData("Power",outtakeOperator.hoodPID.power);
//        dashboardtelemetry.addData("Error Hood",((66.81-Double.parseDouble(output.get("angle")))/outtakeOperator.servoDegPerRot*outtakeOperator.ticksPerRevHood)-outtakeOperator.hoodEncoder.getCurrentPosition());
//        dashboardtelemetry.addData("Power Hood",outtakeOperator.hoodPID.power);
//        dashboardtelemetry.addData("P",outtakeOperator.hoodPID.Pd);
//        dashboardtelemetry.addData("I",outtakeOperator.hoodPID.Id);
//        dashboardtelemetry.addData("D",outtakeOperator.hoodPID.Dd);
//        if (hoodState==HoodState.AUTO && outtakeOperator.apriltag.hasValidTarget()){
        if (hoodState==HoodState.AUTO){
            telemetry.addData("Velocity",output.get("velocity"));
            telemetry.addData("Angle",output.get("angle"));
            double tmp=0;
            for (int i=0;i<LLsample.size();i++){
                tmp+=LLsample.get(i);
            }
            telemetry.addData("Average",tmp/LLsample.size());
            telemetry.addData("Size",LLsample.size());
            telemetry.addData("Total",tmp);
            if (LLsampleTimer.milliseconds()>=50){
                if (LLsample.size()<=MAX_SAMPLE_SIZE){
                    LLsample.add(outtakeOperator.getDistance());
                }else{
                    LLsample.removeFirst();
                    LLsample.add(outtakeOperator.getDistance());
                }
            }
            if (timer.milliseconds()>200) {
                double areaSum=0;
                for (int i=0;i<LLsample.size();i++){
                    areaSum+=LLsample.get(i);
                }
                if (LLsample.size()>=MIN_SAMPLE_SIZE) {
                    output = outtakeOperator.findOptimalLaunch(areaSum / LLsample.size());
                }else{
                    output=null;
                }
                if (output != null) {
                    output.putIfAbsent("velocity", "0.0");
                    output.putIfAbsent("angle", "0.0");
                } else {
                    output = Map.of("velocity", "0.0", "angle", "0.0");
                }
                if (Double.parseDouble(output.get("velocity"))>=0) {
                    flywheelSpeed = Double.parseDouble(output.get("velocity"));
                }
                timer.reset();
            }
            if (Double.parseDouble(output.get("angle"))>=0){
                //9000 max
                //0 min
                outtakeOperator.setHoodEncoder(Double.parseDouble(output.get("angle")));
            }
//            if (outtakeOperator.hoodEncoder.getCurrentPosition()>=9400){
//                hoodServo.setPower(0);
//            }
        }else{
            updateHoodControl();
        }
        if (turretState==TurretState.AUTO){
//            outtakeOperator.setPipeLine(5);
            outtakeOperator.autoturn();
        }else{
            outtakeOperator.turretServo.setPower(gamepad2.left_trigger-gamepad2.right_trigger);
        }
    }

    @Override
    public void stop() {
//        //Save testing logs
//        ReadWriteFile.writeFile(file, data.toString());
        // Clean shutdown
        flywheel.setVelocity(0);
        flywheelR.setVelocity(0);
        intake.setPower(0);
        hoodServo.setPower(0);
    }
}