package org.firstinspires.ftc.teamcode.Brian.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Alvin.intake;
import org.firstinspires.ftc.teamcode.Brian.spindexerColor;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.opModeDataTransfer;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtakeV3FittedAutolaunch;

import java.util.Map;

@Autonomous
public class AutonBlue extends LinearOpMode {
    outtakeV3FittedAutolaunch outtake;
    intake intakeSystem;
    spindexerColor spindexer;
    CRServo spindexerServo=null;
    CRServo turret=null;
    DcMotor intakeMotor=null;
    CRServo hood=null;
    Pose2d beginPose=new Pose2d(-57.5, -43.5, Math.toRadians(-126));
    MecanumDrive drive=null;
    NormalizedColorSensor intakeSensor;
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;
    boolean pauseSpindexer=false;
    boolean turretAligned=false;
    boolean disableTurret=false;
    boolean toggleTurret =false;
    int intakecnt=0;
    enum SpindexerState {
        STOPPED,
        HOLDING,
        INTAKE
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize subsystems
        spindexerServo=hardwareMap.get(CRServo.class,"spindexerServo");
        intakeMotor=hardwareMap.dcMotor.get("intake");
//        transfer=(DcMotorEx) hardwareMap.dcMotor.get("par1");
//        flywheel=(DcMotorEx) hardwareMap.dcMotor.get("flywheel");
//        flywheelR=(DcMotorEx) hardwareMap.dcMotor.get("flywheelR");
//        flywheelR.setDirection(DcMotorSimple.Direction.REVERSE);

        hood=hardwareMap.crservo.get("hoodServo");
        hood.setPower(0);
        turret=hardwareMap.get(CRServo.class,"turretServo");
        turret.setPower(0);
        drive=new MecanumDrive(hardwareMap,beginPose);
        intakeSensor=hardwareMap.get(NormalizedColorSensor.class,"intakeSensor");
        outtake = new outtakeV3FittedAutolaunch(hardwareMap,"Blue",true,drive);
        outtake.setPipeLine(2);
        intakeSystem = new intake(hardwareMap,"intake","intakeSensor");
        spindexer=new spindexerColor(spindexerServo,intakeMotor,hardwareMap);
        spindexer.spindexerSensor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.spindexerSensor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        final Vector2d shootingPos=new Vector2d(-34,-23);
        final double shootingAngle=Math.toRadians(-140);
        final double intakeFinishy = -50;
        final double intakeStarty=-13;
        final double waitTime=1;
        final double shootTime=3;
        final double row1XPos=-9;
        final double row2XPos=16;
        final double row3XPos=38;

        dashboard=FtcDashboard.getInstance();
        dashboardTelemetry=dashboard.getTelemetry();
        dashboardTelemetry.addData("Hood: Current Angle",0);
        dashboardTelemetry.addData("Power",0);
        dashboardTelemetry.addData("Voltage",0);
        dashboardTelemetry.update();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        outtake.resetHoodAngle();

        waitForStart();


        while (opModeIsActive()) {
            if (isStopRequested()) return;
            Actions.runBlocking(new ParallelAction(
                    drive.actionBuilder(beginPose)
                            //STARTPOSITION IS FACING THE WALL!!
                            //Start Flywheel 0
//                            .stopAndAdd(new SpinFlywheel(1600,70))
                            .stopAndAdd(new SpinFlywheel(9999,50))
                            .strafeToLinearHeading(shootingPos, shootingAngle)
                            //Shooting Sequence 0
//                            .stopAndAdd(new TurretAutoAimUntilAligned(1,75,60,5000))
                            .stopAndAdd(new SetHoodEncoder(6060,75))
                            .stopAndAdd(new awaitSpinFlywheel(1531,25))
                            .stopAndAdd(new TurretAutoAimUntilAligned(0.15,100,60,3500))
                            .stopAndAdd(new transferUp())
                            .stopAndAdd(new RunIntake())
                            .stopAndAdd(new rotateSpindexer())
                            //Stop Sequence 0
//                            .stopAndAdd(new StopFlywheel())
                            .stopAndAdd(new transferOff())
                            .stopAndAdd(new StopIntake())
                            .stopAndAdd(new ToggleSpindexer(false))
                            .stopAndAdd(new toggleTurretAutoAim(false))
                            .build(),new awaitSpinFlywheel(1531,50),new SetHoodEncoder(6060,75)));
            //First intake
            Actions.runBlocking(new ParallelAction(drive.actionBuilder(drive.localizer.getPose())
                    //Start Intake Code 1
                    .strafeToLinearHeading(new Vector2d(row1XPos-5, intakeStarty), Math.toRadians(-90))
//                    .stopAndAdd(new RunIntake())
//                    .stopAndAdd(new startspindexer(1))
                    .stopAndAdd(new SpinFlywheel(1570,60))
                    .strafeTo(new Vector2d(row1XPos,intakeFinishy+5))
                    .stopAndAdd(new awaitSpindexerIntake(2))
                    .strafeTo(new Vector2d(row1XPos,intakeFinishy))
                    .build()
                    ,new SpinToIntake(-1,1)));
            //After first intake
            Actions.runBlocking(new ParallelAction(drive.actionBuilder(drive.localizer.getPose())
                    //Stop Intake 1
                    .stopAndAdd(new stopspindexer())
                    .stopAndAdd(new StopIntake())
                    .stopAndAdd(new stopspindexer())
//                    .stopAndAdd(new toggleDisableTurretAutoAim(true))

                    //Start Flywheel 1
//                    .stopAndAdd(new SpinFlywheel(1833,50))
                    .strafeToLinearHeading(new Vector2d(row1XPos, intakeStarty+10), shootingAngle)
                    //Shoot Sequence 1
//                    .stopAndAdd(new TurretAutoAimUntilAligned(0.8,75,60,5000))
                    .stopAndAdd(new awaitTurretAutoAim(1250))
                    .stopAndAdd(new awaitTurretAutoAim(1250))
                    .stopAndAdd(new transferUp())
                    .stopAndAdd(new RunIntake())
                    .stopAndAdd(new rotateSpindexer())
                    //Stop Sequence 1
//                    .stopAndAdd(new StopFlywheel())
                    .stopAndAdd(new transferOff())
                    .stopAndAdd(new stopspindexer())
                    .stopAndAdd(new StopIntake())
//                    .stopAndAdd(new SetIntakePower(-1))
                    .stopAndAdd(new toggleTurretAutoAim(false))
                    .build(),new TurretAutoAimWhileTrue(0.4,150,60)));
            //Second intake
            Actions.runBlocking(new ParallelAction(drive.actionBuilder(drive.localizer.getPose())
                    //Start Intake 2
                    .strafeToLinearHeading(new Vector2d(row2XPos-2, intakeStarty+5), Math.toRadians(-87))
//                    .stopAndAdd(new StopIntake())
//                    .stopAndAdd(new RunIntake())
                    .strafeTo(new Vector2d(row2XPos, intakeFinishy-5))
                    .strafeTo(new Vector2d(row2XPos, intakeFinishy-7))
//                    .stopAndAdd(new ToggleSpindexer(true))
                    .build()
                    ,new SpinToIntake(20000,1)));
            //After second intake
            Actions.runBlocking(new ParallelAction(drive.actionBuilder(drive.localizer.getPose())
                    .stopAndAdd(new ToggleSpindexer(false))
                    //Stop Intake 2
//                    .waitSeconds(waitTime)
                    .stopAndAdd(new StopIntake())
                    .stopAndAdd(new stopspindexer())
//                            .strafeToLinearHeading(new Vector2d(row2XPos, intakeStarty-7), Math.toRadians(360-270))
                    //Start Flywheel 2
//                    .stopAndAdd(new SpinFlywheel(1833,50))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(new Vector2d(row1XPos, intakeStarty+5),shootingAngle),shootingAngle+Math.toRadians(90))
                    //Shoot Sequence 2
                    .stopAndAdd(new awaitTurretAutoAim(1250))
                    .stopAndAdd(new awaitTurretAutoAim(1250))
                    .stopAndAdd(new transferUp())
                    .stopAndAdd(new RunIntake())
                    .stopAndAdd(new rotateSpindexer())
//                    .waitSeconds(shootTime)
                    //Stop Sequence 2
                    .stopAndAdd(new StopFlywheel())
                    .stopAndAdd(new StopIntake())
                    .stopAndAdd(new transferOff())
                    .stopAndAdd(new stopspindexer())
                    .stopAndAdd(new toggleDisableTurretAutoAim(false))
//                    .stopAndAdd(new SetIntakePower(-1))
                    .build(),new TurretAutoAimWhileTrue(0.4,100,60)));
//            //Third intake
//            Actions.runBlocking(new ParallelAction(drive.actionBuilder(drive.localizer.getPose())
//                    //Start Intake 3
//                    .strafeToLinearHeading(new Vector2d(row3XPos-3, intakeStarty-7), Math.toRadians(360-265))
//                    .stopAndAdd(new StopIntake())
//                    .stopAndAdd(new RunIntake())
//                    .strafeTo(new Vector2d(row3XPos, intakeFinishy+5))
//                    .stopAndAdd(new ToggleSpindexer(true))
//                    .build()
//                    ,new SpinToIntake()));
//            //After third intake
//            Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
//                    .stopAndAdd(new ToggleSpindexer(false))
//                    //Stop Intake 3
//                    .waitSeconds(waitTime)
//                    .stopAndAdd(new StopIntake())
//                    .stopAndAdd(new stopspindexer())
//                    //Start Flywheel 3
//                    .stopAndAdd(new SetIntakePower(-1))
//                    .stopAndAdd(new StopIntake())
//                    .stopAndAdd(new SpinFlywheel(1833,50))
//                    .strafeToLinearHeading(new Vector2d(row1XPos, intakeStarty-10), shootingAngle)
//                    //Shoot Sequence 3
//                    .stopAndAdd(new TurretAutoAimUntilAligned())
//                    .stopAndAdd(new transferUp())
//                    .stopAndAdd(new RunIntake())
//                    .stopAndAdd(new startspindexer())
//                    .stopAndAdd(new TurretAutoAimUntilAligned())
//                    .waitSeconds(shootTime)
//                    //Stop Sequence 3
//                    .stopAndAdd(new StopFlywheel())
//                    .stopAndAdd(new transferOff())
//                    .stopAndAdd(new stopspindexer())
//                    .stopAndAdd(new StopIntake())
//                    .build());
            break;
        }
        opModeDataTransfer.currentPose=drive.localizer.getPose();

    }

    /**
     * Helper method to run an action with timeout
     */
    private void runAction(Action action, long timeoutMs) {
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .stopAndAdd(action)
                        .build()
        );
    }

    // ==================== TURRET ACTION ====================
    /**
     * Auto-aims turret to AprilTag based on team color
     */
    public class ScanMotif implements Action {
        private boolean isComplete = false;

        @Override
        public boolean run(TelemetryPacket packet) {
            if (isComplete) return false;

            // autoturn returns false if no valid target or complete
            boolean hasTarget = outtake.autoturn();

            telemetry.addData("Turret: Has Target", hasTarget);

            // Keep running until aligned
            return true;
        }
    }

    /**
     * Auto-aims turret until within threshold, then completes
     */

    //TODO: Integrate tower recognition into auton and make it have the ability to recognize the tower not the motif
    public class TurretAutoAimUntilAligned implements Action {
        private boolean initialized=false;
        private double alignmentThreshold; // degrees, adjust as needed
        double hoodEpsilon;
        double timeout;
        int flywheelEpsilon;
        ElapsedTime timer=new ElapsedTime();
        Map<String,String> optimalLaunch;

        public TurretAutoAimUntilAligned(double epsilon,double hoodEspilon,int flywheelEpsilon,double timeout) {
            this.timer.reset();
            outtake.turretEpsilon=epsilon;
            alignmentThreshold=epsilon;
            this.hoodEpsilon=hoodEspilon;
            this.flywheelEpsilon=flywheelEpsilon;
            this.timeout=timeout;
        }

        @Override
        public boolean run(TelemetryPacket telemetryPacket) {
            if (this.timer.milliseconds()>=timeout){
                outtake.turretServo.setPower(0);
                outtake.hoodServo.setPower(0);
                return false;
            }
            if (this.timer.milliseconds()%50<=3){
                this.optimalLaunch = outtake.findOptimalLaunch(outtake.getDistance());
            }
            if (!initialized){
                initialized=true;
                outtake.turnPID.init();
                outtake.hoodPID.init();
                outtake.epsilonHood=this.hoodEpsilon;
                outtake.maxpower=0.75;
                outtake.minpower=-outtake.maxpower;
                this.optimalLaunch = outtake.findOptimalLaunch(outtake.getDistance());
            }

            boolean hasTarget = outtake.autoturn();

            // Check if aligned by examining heading error
            double headingError = Math.abs(outtake.apriltag.getYaw());
            boolean flywheelAtSpeed=outtake.spin_flywheel(Double.parseDouble(this.optimalLaunch.get("velocity")),this.flywheelEpsilon);
            boolean hoodPos = outtake.setHoodEncoder(Double.parseDouble(this.optimalLaunch.get("angle")));
            telemetry.addData("hoodtarget",Double.parseDouble(this.optimalLaunch.get("angle")));
            telemetry.addData("Current",outtake.hoodEncoder.getCurrentPosition());
            telemetry.update();

            if (!hasTarget) {
                telemetry.addData("Turret: Status", "No Target");
                // Optionally complete after some attempts or keep trying
                return true;
            }
            if (headingError < alignmentThreshold && hoodPos && flywheelAtSpeed) {
                outtake.turretServo.setPower(0); // Stop the turret
                telemetry.addData("Turret: Status", "Aligned!");
                outtake.hoodServo.setPower(0);
//                return true;
                return false; // Action complete
            }

            telemetry.addData("Turret: Status", "Aligning");
            return true;
        }
    }

    public class awaitSpindexerIntake implements Action{
        int cnt;
        public awaitSpindexerIntake(int cnt){
            this.cnt=cnt;
        }
        @Override
        public boolean run(TelemetryPacket telemetryPacket){
            return intakecnt==this.cnt;
        }
    }

    public class toggleTurretAutoAim implements Action{
        boolean toggle;
        public toggleTurretAutoAim(boolean toggle){
            this.toggle=toggle;
        }
        @Override
        public boolean run(TelemetryPacket telemetryPacket){
            toggleTurret =!toggle;
            return false;
        }
    }

    public class toggleDisableTurretAutoAim implements Action{
        boolean toggle;
        public toggleDisableTurretAutoAim(boolean toggle){
            this.toggle=toggle;
        }
        @Override
        public boolean run(TelemetryPacket telemetryPacket){
            disableTurret =toggle;
            return false;
        }
    }

    public class awaitTurretAutoAim implements Action{
        ElapsedTime timer=new ElapsedTime();
        double timeout;
        public awaitTurretAutoAim(double timeout){
            this.timeout=timeout;
            timer.reset();
        }
        @Override
        public boolean run(TelemetryPacket telemetryPacket){
            if (timer.milliseconds()>=timeout){
                return false;
            }
            return !turretAligned;
        }
    }

    /**
     * Auto-aims turret & autoadjust, sets
     */

    public class TurretAutoAimWhileTrue implements Action {
        private boolean initialized=false;
        private double alignmentThreshold; // degrees, adjust as needed
        double hoodEpsilon;
        int flywheelEpsilon;
        ElapsedTime timer=new ElapsedTime();
        Map<String,String> optimalLaunch;

        public TurretAutoAimWhileTrue(double epsilon,double hoodEspilon,int flywheelEpsilon) {
            this.timer.reset();
            toggleTurret =false;
            turretAligned=false;
            disableTurret=false;
            outtake.turretEpsilon=epsilon;
            alignmentThreshold=epsilon;
            this.hoodEpsilon=hoodEspilon;
            this.flywheelEpsilon=flywheelEpsilon;
        }

        @Override
        public boolean run(TelemetryPacket telemetryPacket) {
            if (toggleTurret){
                outtake.turretServo.setPower(0);
                outtake.hoodServo.setPower(0);
                return false;
            }
            if (disableTurret){
                outtake.turretServo.setPower(0);
                outtake.hoodServo.setPower(0);
                return true;
            }
            if (this.timer.milliseconds()%50<=3){
                this.optimalLaunch = outtake.findOptimalLaunch(outtake.getDistance());
            }
            if (!initialized){
                initialized=true;
                outtake.turnPID.init();
                outtake.hoodPID.init();
                outtake.epsilonHood=this.hoodEpsilon;
                outtake.maxpower=0.75;
                outtake.minpower=-outtake.maxpower;
                this.optimalLaunch = outtake.findOptimalLaunch(outtake.getDistance());
            }

            boolean hasTarget = outtake.autoturn();

            // Check if aligned by examining heading error
            double headingError = Math.abs(outtake.apriltag.getYaw());
            boolean flywheelAtSpeed=outtake.spin_flywheel(Double.parseDouble(this.optimalLaunch.get("velocity")),this.flywheelEpsilon);
            boolean hoodPos = outtake.setHoodEncoder(Double.parseDouble(this.optimalLaunch.get("angle")));
            telemetry.addData("hoodtarget",Double.parseDouble(this.optimalLaunch.get("angle")));
            telemetry.addData("Current",outtake.hoodEncoder.getCurrentPosition());
            telemetry.update();

            if (!hasTarget){
                telemetry.addData("Turret: Status", "No Target");
                // Optionally complete after some attempts or keep trying
                return true;
            }
            if (headingError < alignmentThreshold && hoodPos && flywheelAtSpeed) {
                outtake.turretServo.setPower(0); // Stop the turret
                telemetry.addData("Turret: Status", "Aligned!");
                outtake.hoodServo.setPower(0);
                turretAligned=true;
            }else{
                turretAligned=false;
            }

            telemetry.addData("Turret: Status", "Aligning");
            return true;
        }
    }

    public class StopTurret implements Action {
        @Override
        public boolean run(TelemetryPacket packet) {
            outtake.turretServo.setPower(0);
            telemetry.addData("Turret: Status", "Stopped");
            return false; // Complete immediately
        }
    }

    public class initHood implements Action{

        @Override
        public boolean run(TelemetryPacket packet){
            outtake.initHoodAngleBlocking();
            return false;
        }
    }

    // ==================== HOOD ACTION ====================
    /**
     * Sets hood to specific angle and waits until reached
     */
    public class SetHoodAngle implements Action {
        private final double targetAngle;
        private boolean started = false;
        double epsilon=1;

        public SetHoodAngle(double angleDegrees) {
            this.targetAngle = angleDegrees;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!started) {
                started = true;
//                telemetry.addData("Hood: Target Angle", targetAngle);
//                telemetry.update();
//                outtake.initHoodAngleBlocking();
//                outtake.hoodEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//                while(outtake.hoodEncoder.getCurrentPosition()>=-3*outtake.ticksPerRevHood){
//                    telemetry.addData("position",outtake.hoodEncoder.getCurrentPosition());
//                    telemetry.update();
//                    outtake.hoodServo.setPower(1);
//                }
//                outtake.hoodServo.setPower(0);
//                outtake.hoodEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                outtake.hoodPID.init();
//                telemetry.update();
//                outtake.updateHoodAngle();
            }

            // Update hood position
            boolean atPosition = outtake.setHood(targetAngle);

            telemetry.addData("Hood: Target", (66.81 - targetAngle) / outtake.servoDegPerRot * outtake.ticksPerRevHood);
            telemetry.addData("Hood: Current Angle", outtake.hoodEncoder.getCurrentPosition());
            telemetry.addData("Power", outtake.hoodPID.power);
            dashboardTelemetry.addData("Hood: Target", (66.81 - targetAngle) / outtake.servoDegPerRot * outtake.ticksPerRevHood);
            dashboardTelemetry.addData("Hood: Current Angle", outtake.hoodEncoder.getCurrentPosition());
            dashboardTelemetry.addData("Power", outtake.hoodPID.power);
            dashboardTelemetry.update();
            telemetry.addData("Hood: At Position", atPosition);
            telemetry.update();

            // Return false when at position (action complete)
            return !(atPosition);
        }
    }

    /**
     * Sets hood to specific encoder and waits until reached
     */
    public class SetHoodEncoder implements Action {
        private final double targetAngle;
        private boolean started = false;

        public SetHoodEncoder(double angleDegrees, double epsilon) {
            this.targetAngle = angleDegrees;
            outtake.epsilonHood=epsilon;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!started) {
                started = true;
//                telemetry.addData("Hood: Target Angle", targetAngle);
//                telemetry.update();
//                outtake.initHoodAngleBlocking();
//                outtake.hoodEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//                while(outtake.hoodEncoder.getCurrentPosition()>=-3*outtake.ticksPerRevHood){
//                    telemetry.addData("position",outtake.hoodEncoder.getCurrentPosition());
//                    telemetry.update();
//                    outtake.hoodServo.setPower(1);
//                }
//                outtake.hoodServo.setPower(0);
//                outtake.hoodEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                outtake.hoodPID.init();
//                telemetry.update();
//                outtake.updateHoodAngle();
            }

            // Update hood position
            boolean atPosition = outtake.setHoodEncoder(targetAngle);

            // Return false when at position (action complete)
            if (atPosition){
                outtake.hoodServo.setPower(0);
            }
            return !(atPosition);
        }
    }

    // ==================== FLYWHEEL ACTION ====================
    /**
     * Spins flywheel to target speed and waits until stable
     */
    public class SpinFlywheel implements Action {
        private final double targetSpeed;
        private final int tolerance;
        private boolean started = false;

        public SpinFlywheel(double targetSpeedTicksPerSec, int toleranceTicksPerSec) {
            this.targetSpeed = targetSpeedTicksPerSec;
            this.tolerance = toleranceTicksPerSec;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!started) {
                started = true;
                telemetry.addData("Flywheel: Target Speed", targetSpeed);
            }

            // Spin and check if at speed
            boolean atSpeed = outtake.spin_flywheel(targetSpeed, tolerance);

            telemetry.addData("Flywheel: Current Speed", outtake.flywheelDriveR.getVelocity());
            telemetry.addData("Flywheel: At Speed", atSpeed);

            // Return false when at speed (action complete)
            return false;
        }
    }

    /**
     * Spins flywheel to target speed and waits until stable
     */
    public class awaitSpinFlywheel implements Action {
        private final double targetSpeed;
        private final int tolerance;
        private boolean started = false;

        public awaitSpinFlywheel(double targetSpeedTicksPerSec, int toleranceTicksPerSec) {
            this.targetSpeed = targetSpeedTicksPerSec;
            this.tolerance = toleranceTicksPerSec;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!started) {
                started = true;
                telemetry.addData("Flywheel: Target Speed", targetSpeed);
            }

            // Spin and check if at speed
            boolean atSpeed = outtake.spin_flywheel(targetSpeed, tolerance);

            telemetry.addData("Flywheel: Current Speed", outtake.flywheelDriveR.getVelocity());
            telemetry.addData("Flywheel: At Speed", atSpeed);

            // Return false when at speed (action complete)
            return !atSpeed;
        }
    }

    /**
     * Stops the flywheel
     */
    public class StopFlywheel implements Action {
        @Override
        public boolean run(TelemetryPacket packet) {
            outtake.spin_flywheel(0, 10);
            telemetry.addData("Flywheel: Status", "Stopped");
            return false; // Complete immediately
        }
    }

    // ==================== TRANSFER ACTION ====================
    public class transferUp implements Action {
        @Override
        public boolean run(TelemetryPacket packet) {
            outtake.transferUp();
            telemetry.addData("Transfer: Status", "Up");
            return false; // Complete immediately
        }
    }

    public class transferOff implements Action {
        @Override
        public boolean run(TelemetryPacket packet) {
            outtake.transferDown();
            telemetry.addData("Transfer: Status", "Off");
            return false; // Complete immediately
        }
    }

    // ==================== INTAKE ACTION ====================
    /**
     * Runs intake until pixel detected or timeout
     */
    public class IntakePixel implements Action {
        private final long timeoutMs;

        public IntakePixel(long timeoutMilliseconds) {
            this.timeoutMs = timeoutMilliseconds;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            boolean pixelDetected = intakeSystem.intakeUntilPixel();
            telemetry.addData("Intake: Pixel Detected", pixelDetected);
            telemetry.addData("Intake: Status", pixelDetected ? "Complete" : "Running");
            telemetry.update();
            return !pixelDetected; // Return false when complete
        }
    }

    /**
     * Just runs intake without spindexer
     */
    public class RunIntake implements Action {

        @Override
        public boolean run(TelemetryPacket packet) {
            intakeSystem.start();
            return false;
        }
    }

    public class StopIntake implements Action {

        @Override
        public boolean run(TelemetryPacket packet) {
            intakeSystem.stop();
            return false;
        }
    }

    /**
     * Manually control intake power
     */
    public class SetIntakePower implements Action {
        private final double power;

        public SetIntakePower(double power) {
            this.power = power;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            intakeSystem.setPower(power);
            telemetry.addData("Intake: Power", power);
            return false; // Complete immediately
        }
    }

    // ==================== SPINDEXER ACTIONS ====================
    /**
     * Spins spindexer to match current motif pattern
     */
    public class SpinToMotif implements Action {
        private final int motifIndex;
        private boolean initialized = false;

        public SpinToMotif(int motifIndex) {
            this.motifIndex = motifIndex;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!initialized) {
                spindexer.initSpin();
                initialized = true;
            }

            boolean complete = spindexer.spinToMotif(motifIndex);

            telemetry.addData("Spindexer: Motif Index", motifIndex);
            telemetry.addData("Spindexer: Status", complete ? "Complete" : "Spinning");

            return !complete; // Return false when complete
        }
    }

    public class startspindexer implements Action {
        double power;
        public startspindexer(double speed){
            this.power=speed;
        }
        @Override
        public boolean run(TelemetryPacket packet) {
            spindexerServo.setPower(power);
            return false;
        }
    }

    public class rotateSpindexer implements Action{
        int startEncoder;
        boolean initialized=false;
        @Override
        public boolean run(TelemetryPacket packet) {
            if (!initialized){
                startEncoder=spindexer.spindexerSensor.getCurrentPosition();
                initialized=true;
            }
            spindexerServo.setPower(1);
            return Math.abs(spindexer.spindexerSensor.getCurrentPosition()-startEncoder)<=8192;
        }
    }

    public class stopspindexer implements Action {
        @Override
        public boolean run(TelemetryPacket packet) {
            spindexerServo.setPower(0);
            return false;
        }
    }


    /**
     * Spins spindexer to next empty slot for intake
     */
    public class SpinToIntake implements Action {
        private boolean initialized = false;
        private ElapsedTime timer=new ElapsedTime();
        private ElapsedTime finishtimer=new ElapsedTime();
        boolean previousStopped=false;
        public double timeout;
        public double intakePower;
        SpindexerState spindexerState = SpindexerState.STOPPED;

        public SpinToIntake(double timeout, double intakePower){
            this.timeout=timeout;
            this.intakePower=intakePower;
            intakecnt=0;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (timeout != -1 && this.timer.milliseconds()>=timeout){
                spindexer.spindexerServo.setPower(0);
                return false;
            }
            if (!initialized) {
                this.timer.reset();
                this.finishtimer.reset();
                spindexer.initSpin();
                spindexerState=SpindexerState.INTAKE;
                initialized = true;
            }
            if (spindexerState==SpindexerState.INTAKE && spindexer.spindexerSensor.getVelocity()<10){
                intakeSystem.setPower(0);
            }else{
                intakeSystem.setPower(this.intakePower);
            }
            if (spindexerState== SpindexerState.INTAKE){
                boolean result=spindexer.spinToIntake();
                if (intakecnt==3){
                    spindexerState=SpindexerState.STOPPED;
                }
                if (result){
                    spindexerState= SpindexerState.HOLDING;
                }else if (spindexer.detectioncnt==3){
                    spindexerState= SpindexerState.STOPPED;
                    gamepad1.rumble(100);
                }
            }
            else if (spindexerState== SpindexerState.STOPPED){
                spindexer.spindexerServo.setPower(0);
                if (!previousStopped) {
                    previousStopped = true;
                    finishtimer.reset();
                    intakeSystem.setPower(this.intakePower);
                }
                if (finishtimer.milliseconds()>=100){
                    intakeSystem.stop();
                    return false;
                }
            }
            else if (spindexerState== SpindexerState.HOLDING){
                spindexer.holdSpindexer();
                if (spindexer.intakesensor.isGreen() || spindexer.intakesensor.isPurple()){
                    spindexerState= SpindexerState.INTAKE;
                    intakecnt++;
                    spindexer.initSpin();
                }
            }
            return true; // Return false when complete
        }
    }

    public class ToggleSpindexer implements Action {
        private boolean onOff;

        public ToggleSpindexer(boolean onOff){
            this.onOff=onOff;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            pauseSpindexer=onOff;
            return false;
        }
    }
}