package org.firstinspires.ftc.teamcode.Stanley.autonPath;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Alvin.colorSensor;
import org.firstinspires.ftc.teamcode.Alvin.intake;
import org.firstinspires.ftc.teamcode.Brian.spindexer;
import org.firstinspires.ftc.teamcode.Brian.spindexerColor;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtakeV2;

@Autonomous
public class AutonRedPathV2 extends LinearOpMode {

    // TODO:Subsystem instances - initialize these in runOpMode
    outtakeV2 outtake;
    intake intakeSystem;
    spindexerColor spindexer;
    CRServo spindexerServo=null;
    ElapsedTime timer=new ElapsedTime();
    DcMotor intakeMotor=null;
    DcMotorEx transfer=null;
    DcMotorEx flywheel=null;
    DcMotorEx flywheelR=null;
    CRServo hood=null;
    AnalogInput hoodSensor=null;
    Pose2d beginPose=new Pose2d(-57.5, 43.5, Math.toRadians(360-54));
    MecanumDrive drive=null;
    NormalizedColorSensor intakeSensor;
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize subsystems
        spindexerServo=hardwareMap.crservo.get("spindexerServo");
        intakeMotor=hardwareMap.dcMotor.get("intake");
        hoodSensor=hardwareMap.get(AnalogInput.class,"hoodAnalog");
        flywheel=(DcMotorEx) hardwareMap.dcMotor.get("flywheel");
        flywheelR=(DcMotorEx) hardwareMap.dcMotor.get("flywheelR");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer=(DcMotorEx) hardwareMap.dcMotor.get("par1");
        drive=new MecanumDrive(hardwareMap,beginPose);
//        transfer=hardwareMap.servo.get("transferServo");
        hood=hardwareMap.crservo.get("hoodServo");
        intakeSensor=hardwareMap.get(NormalizedColorSensor.class,"intakeSensor");
        outtake = new outtakeV2(hardwareMap,flywheel,flywheelR,null,null,null,null,null,hood,hoodSensor,transfer,true);
        intakeSystem = new intake(hardwareMap,"intake","intakeSensor");
        spindexer=new spindexerColor(spindexerServo,intakeMotor,hardwareMap);
        final Vector2d shootingPos=new Vector2d(-34,23);
        final double shootingAngle=Math.toRadians(225);
        final double intakeFinishy =36;
        final double intakeStarty=13;

        dashboard=FtcDashboard.getInstance();
        dashboardTelemetry=dashboard.getTelemetry();
        dashboardTelemetry.addData("Hood: Current Angle",0);
        dashboardTelemetry.update();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) return;
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)


                            .strafeToLinearHeading(shootingPos, shootingAngle)
//                            //TODO: Add Start Flywheel DOOOOOOOOOOOOOONNNNNNNNNNNNNNNEEEEEEEEEEEEE
//                            .stopAndAdd(new SpinFlywheel(670,50))
//                            .waitSeconds(3)
//                            //TODO: Add Shooting Sequence DOOOOOOOOOOOOOONNNNNNNNNNNNNNNEEEEEEEEEEEEE
//                            .stopAndAdd(new transferUp())
//                            .waitSeconds(.5)
//                            .stopAndAdd(new RunIntake())
//                            .stopAndAdd(new startspindexer())
//                            .waitSeconds(.5)
//                            //TODO: Add Stop Sequence DOOOOOOOOOOOOOONNNNNNNNNNNNNNNEEEEEEEEEEEEE
//                            .stopAndAdd(new StopFlywheel())
//                            .stopAndAdd(new transferOff())
//                            .stopAndAdd(new stopspindexer())
//                            .stopAndAdd(new StopIntake())
//                            .waitSeconds(1)
                            .strafeToLinearHeading(new Vector2d(-15, intakeStarty), Math.toRadians(360-270))
                            //TODO: Add Intake Code DOOOOOOOOOOOOOONNNNNNNNNNNNNNNEEEEEEEEEEEEE
//                            .stopAndAdd(new RunIntake())
//                            .stopAndAdd(new startspindexer())
                            .strafeTo(new Vector2d(-15, intakeFinishy))
                            .strafeTo(new Vector2d(-15,intakeFinishy+10))

                            //TODO: Add Stop Intake DOOOOOOOOOOOOOONNNNNNNNNNNNNNNEEEEEEEEEEEEE
//                            .stopAndAdd(new StopIntake())
//                            .waitSeconds(3)
                            .strafeToLinearHeading(shootingPos, shootingAngle)
                            //TODO: Add Start Flywheel DOOOOOOOOOOOOOONNNNNNNNNNNNNNNEEEEEEEEEEEEE
//                            .stopAndAdd(new SpinFlywheel(670,50))
                            .waitSeconds(3)
                            //TODO: Add Shoot Sequence 1 DOOOOOOOOOOOOOONNNNNNNNNNNNNNNEEEEEEEEEEEEE
//                            .stopAndAdd(new transferUp())
//                            .waitSeconds(.5)
//                            .stopAndAdd(new RunIntake())
//                            .stopAndAdd(new startspindexer())
//                            .waitSeconds(.5)
                            //TODO: Add Stop Sequence DOOOOOOOOOOOOOONNNNNNNNNNNNNNNEEEEEEEEEEEEE
//                            .stopAndAdd(new StopFlywheel())
//                            .stopAndAdd(new transferOff())
//                            .stopAndAdd(new stopspindexer())
//                            .stopAndAdd(new StopIntake())
                            .strafeToLinearHeading(new Vector2d(10, intakeStarty-7), Math.toRadians(360-270))
                            //TODO: Add Start Intake DOOOOOOOOOOOOOONNNNNNNNNNNNNNNEEEEEEEEEEEEE
//                            .stopAndAdd(new RunIntake())
                            .strafeTo(new Vector2d(10, intakeFinishy+3))
//                            .waitSeconds(3)
                            //TODO: Add Stop Intake DOOOOOOOOOOOOOONNNNNNNNNNNNNNNEEEEEEEEEEEEE
//                            .stopAndAdd(new StopIntake())
//                            .waitSeconds(3)
                            .strafeToLinearHeading(shootingPos, shootingAngle)
                            //TODO: Add Start Flywheel DOOOOOOOOOOOOOONNNNNNNNNNNNNNNEEEEEEEEEEEEE
//                            .stopAndAdd(new SpinFlywheel(670,50))
//                            .waitSeconds(3)
                            //TODO: Add Shoot Sequence DOOOOOOOOOOOOOONNNNNNNNNNNNNNNEEEEEEEEEEEEE
//                            .stopAndAdd(new transferUp())
//                            .waitSeconds(.5)
//                            .stopAndAdd(new RunIntake())
//                            .waitSeconds(.5)
                            //TODO: Add Stop Sequence DOOOOOOOOOOOOOONNNNNNNNNNNNNNNEEEEEEEEEEEEE
//                            .stopAndAdd(new StopFlywheel())
//                            .stopAndAdd(new transferOff())
//                            .stopAndAdd(new stopspindexer())
//                            .stopAndAdd(new StopIntake())
                            //TODO: Add Start Intake DOOOOOOOOOOOOOONNNNNNNNNNNNNNNEEEEEEEEEEEEE
                            .strafeToLinearHeading(new Vector2d(35, intakeStarty-10), Math.toRadians(360-270))
//                            .stopAndAdd(new RunIntake())
                            .strafeTo(new Vector2d(35, intakeFinishy+3))
//                            .waitSeconds(3)
                            //TODO: Add Stop Intake DOOOOOOOOOOOOOONNNNNNNNNNNNNNNEEEEEEEEEEEEE
//                            .stopAndAdd(new StopIntake())
//                            .waitSeconds(3)
                            .strafeToLinearHeading(shootingPos, shootingAngle)
                            //TODO: Add Start Flywheel DOOOOOOOOOOOOOONNNNNNNNNNNNNNNEEEEEEEEEEEEE
//                            .stopAndAdd(new SpinFlywheel(670,50))
//                            .waitSeconds(3)
                            //TODO: Add Shoot Sequence
//                            .stopAndAdd(new transferUp())
//                            .waitSeconds(.5)
//                            .stopAndAdd(new RunIntake())
//                            .stopAndAdd(new startspindexer())
//                            .waitSeconds(.5)
                            .build());
            break;
        }

        // ===== TEST 1: Turret Auto-Aim ===== DDDDOOOOOOOOOOOOOOOONNNNNNNNNNNNEEEEEEEEEEE
//        telemetry.addData("Test", "1. Turret Auto-Aim");
//        telemetry.update();
//        Action turretAim = new TurretAutoAimUntilAligned(5.0);
//        runAction(turretAim, 3000); // 3 second timeout

//        // ===== TEST 2: Hood Angle =====
//        telemetry.addData("Test", "2. Hood Angle");
//        telemetry.update();
//        Action setHood = new SetHoodAngle(45.0);
//        runAction(setHood, 2000);
//
//        // ===== TEST 3: Flywheel Spin ===== DDDDOOOOOOOOOOOOOOOONNNNNNNNNNNNEEEEEEEEEEE
//        telemetry.addData("Test", "3. Flywheel");
//        telemetry.update();
//        Action spinFlywheel = new SpinFlywheel(2000, 50);
//        runAction(spinFlywheel, 3000);
//
//        // ===== TEST 4: Stop Flywheel ===== DDDDOOOOOOOOOOOOOOOONNNNNNNNNNNNEEEEEEEEEEE
//        telemetry.addData("Test", "4. Stop Flywheel");
//        telemetry.update();
//        Action stopFlywheel = new StopFlywheel();
//        runAction(stopFlywheel, 500);
//
//        // ===== TEST 5: Intake ===== DDDDOOOOOOOOOOOOOOOONNNNNNNNNNNNEEEEEEEEEEE
//        telemetry.addData("Test", "5. Intake Pixel");
//        telemetry.update();
//        Action intake = new IntakePixel(3000);
//        runAction(intake, 3000);

          // ===== TEST 6: Spindexer =====
//        telemetry.addData("Test", "6. Spindexer to Motif");
//        telemetry.update();
//        Action spinMotif = new SpinToMotif(0);
//        runAction(spinMotif, 3000);
//        sleep(500);

          // ===== TEST 7: Spin Transfer =====
//        telemetry.addData("Test", "7. Transfer");
//        telemetry.update();
//        Action spinTransfer = new transferUp();
//        runAction(spinTransfer);

          // ===== TEST 8: Stop Transfer =====
//        telemetry.addData("Test", "8. Transfer");
//        telemetry.update();
//        Action stopTransfer = new transferOff();
//        runAction(stopTransfer);


//        // ===== TEST 9: Full Shoot Sequence =====
//        telemetry.addData("Test", "7. Complete Shoot Sequence");
//        telemetry.update();
//        Action shootSequence = new ShootSequence(45.0, 2000, 50);
//        runAction(shootSequence, 10000);

        telemetry.addData("Status", "All Tests Complete");
        telemetry.update();
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
    public class TurretAutoAim implements Action {
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
    public class TurretAutoAimUntilAligned implements Action {
        private boolean isComplete = false;

        public TurretAutoAimUntilAligned() {
        }

        @Override
        public boolean run(TelemetryPacket telemetryPacket) {
            if (isComplete) return false;

            boolean hasTarget = outtake.autoturn();

            if (!hasTarget) {
                telemetry.addData("Turret: Status", "No Target");
                return true; // Keep trying
            }

            // Check if aligned
            telemetry.addData("Turret: Status", "Aligning");
            return true;
        }
    }

    // ==================== HOOD ACTION ====================
    /**
     * Sets hood to specific angle and waits until reached
     */
    public class SetHoodAngle implements Action {
        private final double targetAngle;
        private boolean started = false;

        public SetHoodAngle(double angleDegrees) {
            this.targetAngle = angleDegrees;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!started) {
                started = true;
                telemetry.addData("Hood: Target Angle", targetAngle);
                outtake.initHoodAngleBlocking();
                outtake.hoodPID.init();
//                outtake.updateHoodAngle();
            }

            // Update hood position
            boolean atPosition = outtake.setHood(targetAngle);

            telemetry.addData("Hood: Current Angle", outtake.hoodAngle* outtake.servoDegPerRot);
            dashboardTelemetry.addData("Hood: Current Angle", outtake.hoodAngle* outtake.servoDegPerRot);
            dashboardTelemetry.update();
            telemetry.addData("Hood: At Position", atPosition);
            telemetry.update();

            // Return false when at position (action complete)
            return true;
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
        @Override
        public boolean run(TelemetryPacket packet) {
            spindexerServo.setPower(1);
            return false;
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
        private final int motifIndex;
        private boolean initialized = false;

        public SpinToIntake(int motifIndex) {
            this.motifIndex = motifIndex;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!initialized) {
                spindexer.initSpin();
                initialized = true;
            }

            boolean complete = spindexer.spinToIntake(motifIndex);

            telemetry.addData("Spindexer: Motif Index", motifIndex);
            telemetry.addData("Spindexer: Status", complete ? "Complete" : "Spinning");

            return !complete; // Return false when complete
        }
    }

    // ==================== COMPOSITE ACTIONS ====================
    /**
     * Complete shooting sequence: aim turret, adjust hood, spin flywheel, then transfer
     */
    public class ShootSequence implements Action {
        private static final int PHASE_AIM = 0;
        private static final int PHASE_HOOD = 1;
        private static final int PHASE_FLYWHEEL = 2;
        private static final int PHASE_TRANSFER = 3;
        private static final int PHASE_COMPLETE = 4;

        private int currentPhase = PHASE_AIM;

        private final double hoodAngle;
        private final double flywheelSpeed;
        private final int flywheelTolerance;

        public ShootSequence(double hoodAngleDegrees, double flywheelSpeedTPS, int tolerance) {
            this.hoodAngle = hoodAngleDegrees;
            this.flywheelSpeed = flywheelSpeedTPS;
            this.flywheelTolerance = tolerance;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            String phaseName = "";
            switch (currentPhase) {
                case PHASE_AIM: phaseName = "AIM"; break;
                case PHASE_HOOD: phaseName = "HOOD"; break;
                case PHASE_FLYWHEEL: phaseName = "FLYWHEEL"; break;
                case PHASE_TRANSFER: phaseName = "TRANSFER"; break;
                case PHASE_COMPLETE: phaseName = "COMPLETE"; break;
            }
            telemetry.addData("Shoot Sequence: Phase", phaseName);

            switch (currentPhase) {
                case PHASE_AIM:
                    boolean hasTarget = outtake.autoturn();
                    if (hasTarget) {
                        // TODO: Check if actually aligned, not just has target
                        currentPhase = PHASE_HOOD;
                    }
                    return true;

                case PHASE_HOOD:
                    boolean hoodReady = outtake.setHood(hoodAngle);
                    if (hoodReady) {
                        currentPhase = PHASE_FLYWHEEL;
                    }
                    return true;

                case PHASE_FLYWHEEL:
                    boolean flywheelReady = outtake.spin_flywheel(flywheelSpeed, flywheelTolerance);
                    if (flywheelReady) {
                        currentPhase = PHASE_TRANSFER;
                        outtake.transferUp();
                    }
                    return true;

                case PHASE_TRANSFER:
                    outtake.transferDown();
                    currentPhase = PHASE_COMPLETE;
                    return true;

                case PHASE_COMPLETE:
                    return false;
            }
            return false;
        }
    }
}