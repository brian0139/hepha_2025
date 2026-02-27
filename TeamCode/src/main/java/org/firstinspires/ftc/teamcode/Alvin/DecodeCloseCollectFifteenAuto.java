package org.firstinspires.ftc.teamcode.Alvin;
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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Alvin.intake;
import org.firstinspires.ftc.teamcode.Brian.spindexerColor;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.opModeDataTransfer;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtakeV3FittedAutolaunch;

@Autonomous(name = "Decode Close Collect 15", group = "Autonomous")
public class DecodeCloseCollectFifteenAuto extends LinearOpMode {
    outtakeV3FittedAutolaunch outtake;
    intake intakeSystem;
    spindexerColor spindexer;
    CRServo spindexerServo = null;
    ElapsedTime timer = new ElapsedTime();
    DcMotor intakeMotor = null;
    DcMotorEx transfer = null;
    DcMotorEx flywheel = null;
    DcMotorEx flywheelR = null;
    CRServo hood = null;
    Pose2d beginPose = new Pose2d(-57.5, -43.5, Math.toRadians(-126)); // Close side starting position
    MecanumDrive drive = null;
    NormalizedColorSensor intakeSensor;
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;
    boolean pauseSpindexer = false;

    private boolean shouldAbortActions() {
        return isStopRequested() || (isStarted() && !opModeIsActive());
    }

    private void runBlockingAbortable(Action action) {
        Actions.runBlocking(new AbortOnStopAction(action));
    }

    private class AbortOnStopAction implements Action {
        private final Action inner;

        private AbortOnStopAction(Action inner) {
            this.inner = inner;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (shouldAbortActions()) {
                return false;
            }
            return inner.run(packet);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize subsystems
        spindexerServo = hardwareMap.get(CRServo.class, "spindexerServo");
        intakeMotor = hardwareMap.dcMotor.get("intake");
        transfer = (DcMotorEx) hardwareMap.dcMotor.get("par1");
        flywheel = (DcMotorEx) hardwareMap.dcMotor.get("flywheel");
        flywheelR = (DcMotorEx) hardwareMap.dcMotor.get("flywheelR");
        hood = hardwareMap.crservo.get("hoodServo");
        drive = new MecanumDrive(hardwareMap, beginPose);
        intakeSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeSensor");

        // Close side turret setup
        outtake = new outtakeV3FittedAutolaunch(hardwareMap, "Red", true, drive);
        outtake.setPipeLine(2);
        intakeSystem = new intake(hardwareMap, "intake", "intakeSensor");
        spindexer = new spindexerColor(spindexerServo, intakeMotor, hardwareMap);

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        // Close-side shooting position
        final Vector2d shootingPos = new Vector2d(-34, -23);
        final double shootingAngle = Math.toRadians(-140);

        // Ramp-focused close cycles (5 x 3 balls = 15 total target)
        final double intakeFinishy = -53;
        final double waitTime = 0.9;
        final double shootTime = 1.5;
        final double rampEntryX = 52;
        final double rampDeepX = 59;
        final Vector2d cycleShootPos = new Vector2d(-7, -3);

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        dashboardTelemetry.addData("Hood: Current Angle", 0);
        dashboardTelemetry.addData("Power", 0);
        dashboardTelemetry.addData("Voltage", 0);
        dashboardTelemetry.update();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize hood
        runBlockingAbortable(drive.actionBuilder(beginPose)
                .stopAndAdd(new initHood())
                .stopAndAdd(new SetHoodAngle(45.2))
                .build());

        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            // Sequence 0: Shoot preloaded 3 balls
            runBlockingAbortable(
                    drive.actionBuilder(beginPose)
                            .stopAndAdd(new SpinFlywheel(1600, 50))
                            .strafeToLinearHeading(shootingPos, shootingAngle)
                            .stopAndAdd(new TurretAutoAimUntilAligned())
                            .stopAndAdd(new transferUp())
                            .stopAndAdd(new RunIntake())
                            .stopAndAdd(new startspindexer())
                            .waitSeconds(shootTime)
                            .stopAndAdd(new StopFlywheel())
                            .stopAndAdd(new transferOff())
                            .stopAndAdd(new stopspindexer())
                            .stopAndAdd(new StopIntake())
                            .stopAndAdd(new ToggleSpindexer(false))
                            .build());

            // Cycle 1 intake from ramp
            runBlockingAbortable(new ParallelAction(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(rampEntryX, -12), Math.toRadians(-90))
                            .stopAndAdd(new StopIntake())
                            .stopAndAdd(new RunIntake())
                            .strafeTo(new Vector2d(rampEntryX, intakeFinishy))
                            .stopAndAdd(new ToggleSpindexer(true))
                            .build(),
                    new SpinToIntake()));

            // Cycle 1 shoot
            runBlockingAbortable(drive.actionBuilder(drive.localizer.getPose())
                    .stopAndAdd(new ToggleSpindexer(false))
                    .waitSeconds(waitTime)
                    .stopAndAdd(new StopIntake())
                    .stopAndAdd(new stopspindexer())
                    .stopAndAdd(new SetHoodEncoder(8020))
                    .stopAndAdd(new SpinFlywheel(1833, 50))
                    .strafeToLinearHeading(cycleShootPos, shootingAngle)
                    .stopAndAdd(new TurretAutoAimUntilAligned())
                    .stopAndAdd(new transferUp())
                    .stopAndAdd(new RunIntake())
                    .stopAndAdd(new startspindexer())
                    .waitSeconds(shootTime)
                    .stopAndAdd(new StopFlywheel())
                    .stopAndAdd(new transferOff())
                    .stopAndAdd(new stopspindexer())
                    .stopAndAdd(new StopIntake())
                    .stopAndAdd(new SetIntakePower(-1))
                    .build());

            // Cycle 2 intake from ramp (deeper pass)
            runBlockingAbortable(new ParallelAction(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(rampDeepX, -16), Math.toRadians(-90))
                            .stopAndAdd(new StopIntake())
                            .stopAndAdd(new RunIntake())
                            .strafeTo(new Vector2d(rampDeepX, intakeFinishy))
                            .stopAndAdd(new ToggleSpindexer(true))
                            .build(),
                    new SpinToIntake()));

            // Cycle 2 shoot
            runBlockingAbortable(drive.actionBuilder(drive.localizer.getPose())
                    .stopAndAdd(new ToggleSpindexer(false))
                    .waitSeconds(waitTime)
                    .stopAndAdd(new StopIntake())
                    .stopAndAdd(new stopspindexer())
                    .stopAndAdd(new SpinFlywheel(1833, 50))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(cycleShootPos, shootingAngle),
                            shootingAngle + Math.toRadians(-90))
                    .setReversed(false)
                    .stopAndAdd(new TurretAutoAimUntilAligned())
                    .stopAndAdd(new transferUp())
                    .stopAndAdd(new RunIntake())
                    .stopAndAdd(new startspindexer())
                    .waitSeconds(shootTime)
                    .stopAndAdd(new StopFlywheel())
                    .stopAndAdd(new StopIntake())
                    .stopAndAdd(new transferOff())
                    .stopAndAdd(new stopspindexer())
                    .stopAndAdd(new SetIntakePower(-1))
                    .build());

            // Cycle 3 intake from ramp
            runBlockingAbortable(new ParallelAction(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(rampEntryX, -20), Math.toRadians(-90))
                            .stopAndAdd(new StopIntake())
                            .stopAndAdd(new RunIntake())
                            .strafeTo(new Vector2d(rampEntryX, intakeFinishy))
                            .stopAndAdd(new ToggleSpindexer(true))
                            .build(),
                    new SpinToIntake()));

            // Cycle 3 shoot
            runBlockingAbortable(drive.actionBuilder(drive.localizer.getPose())
                    .stopAndAdd(new ToggleSpindexer(false))
                    .waitSeconds(waitTime)
                    .stopAndAdd(new StopIntake())
                    .stopAndAdd(new stopspindexer())
                    .stopAndAdd(new SetIntakePower(-1))
                    .stopAndAdd(new StopIntake())
                    .stopAndAdd(new SpinFlywheel(1833, 50))
                    .strafeToLinearHeading(cycleShootPos, shootingAngle)
                    .stopAndAdd(new TurretAutoAimUntilAligned())
                    .stopAndAdd(new transferUp())
                    .stopAndAdd(new RunIntake())
                    .stopAndAdd(new startspindexer())
                    .waitSeconds(shootTime)
                    .stopAndAdd(new StopFlywheel())
                    .stopAndAdd(new transferOff())
                    .stopAndAdd(new stopspindexer())
                    .stopAndAdd(new SetIntakePower(-1))
                    .build());

            // Cycle 4 intake from ramp (deeper pass)
            runBlockingAbortable(new ParallelAction(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(rampDeepX, -24), Math.toRadians(-90))
                            .stopAndAdd(new StopIntake())
                            .stopAndAdd(new RunIntake())
                            .strafeTo(new Vector2d(rampDeepX, intakeFinishy))
                            .stopAndAdd(new ToggleSpindexer(true))
                            .build(),
                    new SpinToIntake()));

            // Cycle 4 shoot
            runBlockingAbortable(drive.actionBuilder(drive.localizer.getPose())
                    .stopAndAdd(new ToggleSpindexer(false))
                    .waitSeconds(waitTime)
                    .stopAndAdd(new StopIntake())
                    .stopAndAdd(new stopspindexer())
                    .stopAndAdd(new SetIntakePower(-1))
                    .stopAndAdd(new SpinFlywheel(1833, 50))
                    .strafeToLinearHeading(cycleShootPos, shootingAngle)
                    .stopAndAdd(new TurretAutoAimUntilAligned())
                    .stopAndAdd(new transferUp())
                    .stopAndAdd(new RunIntake())
                    .stopAndAdd(new startspindexer())
                    .waitSeconds(shootTime)
                    .stopAndAdd(new StopFlywheel())
                    .stopAndAdd(new transferOff())
                    .stopAndAdd(new stopspindexer())
                    .stopAndAdd(new SetIntakePower(-1))
                    .build());

            // Cycle 5 intake from ramp
            runBlockingAbortable(new ParallelAction(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(rampEntryX, -28), Math.toRadians(-90))
                            .stopAndAdd(new StopIntake())
                            .stopAndAdd(new RunIntake())
                            .strafeTo(new Vector2d(rampEntryX, intakeFinishy))
                            .stopAndAdd(new ToggleSpindexer(true))
                            .build(),
                    new SpinToIntake()));

            // Cycle 5 shoot (15 total) then hold near close wing like the recording
            runBlockingAbortable(drive.actionBuilder(drive.localizer.getPose())
                    .stopAndAdd(new ToggleSpindexer(false))
                    .waitSeconds(waitTime)
                    .stopAndAdd(new StopIntake())
                    .stopAndAdd(new stopspindexer())
                    .stopAndAdd(new SetIntakePower(-1))
                    .stopAndAdd(new SpinFlywheel(1833, 50))
                    .strafeToLinearHeading(cycleShootPos, shootingAngle)
                    .stopAndAdd(new TurretAutoAimUntilAligned())
                    .stopAndAdd(new transferUp())
                    .stopAndAdd(new RunIntake())
                    .stopAndAdd(new startspindexer())
                    .waitSeconds(shootTime)
                    .stopAndAdd(new StopFlywheel())
                    .stopAndAdd(new transferOff())
                    .stopAndAdd(new stopspindexer())
                    .stopAndAdd(new StopIntake())
                    .strafeToLinearHeading(new Vector2d(52, -40), Math.toRadians(-90))
                    .build());

            break;
        }
        opModeDataTransfer.currentPose = drive.localizer.getPose();
    }

    // ==================== ACTION CLASSES ====================
    // (All the same action classes from your original code)

    public class ScanMotif implements Action {
        private boolean isComplete = false;

        @Override
        public boolean run(TelemetryPacket packet) {
            if (isComplete) return false;
            boolean hasTarget = outtake.autoturn();
            telemetry.addData("Turret: Has Target", hasTarget);
            return true;
        }
    }

    public class TurretAutoAimUntilAligned implements Action {
        private boolean initialized = false;
        private boolean isComplete = false;
        private final double alignmentThreshold = 2;
        ElapsedTime timer = new ElapsedTime();

        public TurretAutoAimUntilAligned() {
            this.timer.reset();
        }

        @Override
        public boolean run(TelemetryPacket telemetryPacket) {
            if (isComplete) return false;
            if (timer.milliseconds() >= 3500) {
                outtake.turretServo.setPower(0);
                return false;
            }
            if (!initialized) {
                initialized = true;
                outtake.turnPID.init();
            }

            boolean hasTarget = outtake.autoturn();

            if (!hasTarget) {
                telemetry.addData("Turret: Status", "No Target");
                return true;
            }

            double headingError = Math.abs(outtake.apriltag.getYaw());
            if (headingError < alignmentThreshold) {
                outtake.turretServo.setPower(0);
                isComplete = true;
                telemetry.addData("Turret: Status", "Aligned!");
                return false;
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
            return false;
        }
    }

    public class initHood implements Action {
        @Override
        public boolean run(TelemetryPacket packet) {
            outtake.initHoodAngleBlocking();
            return false;
        }
    }

    public class SetHoodAngle implements Action {
        private final double targetAngle;
        private boolean started = false;
        double epsilon = 1;

        public SetHoodAngle(double angleDegrees) {
            this.targetAngle = angleDegrees;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!started) {
                started = true;
                outtake.hoodPID.init();
            }

            boolean atPosition = outtake.setHood(targetAngle);

            telemetry.addData("Hood: Target", (66.81 - targetAngle) / outtake.servoDegPerRot * outtake.ticksPerRevHood);
            telemetry.addData("Hood: Current Angle", outtake.hoodEncoder.getCurrentPosition());
            telemetry.addData("Power", outtake.hoodPID.power);
            telemetry.addData("Hood: At Position", atPosition);
            telemetry.update();

            return !(atPosition);
        }
    }

    public class SetHoodEncoder implements Action {
        private final double targetAngle;
        private boolean started = false;

        public SetHoodEncoder(double angleDegrees) {
            this.targetAngle = angleDegrees;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!started) {
                started = true;
                outtake.hoodPID.init();
            }

            boolean atPosition = outtake.setHoodEncoder(targetAngle);
            return !(atPosition);
        }
    }

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

            boolean atSpeed = outtake.spin_flywheel(targetSpeed, tolerance);

            telemetry.addData("Flywheel: Current Speed", outtake.flywheelDriveR.getVelocity());
            telemetry.addData("Flywheel: At Speed", atSpeed);

            return false;
        }
    }

    public class StopFlywheel implements Action {
        @Override
        public boolean run(TelemetryPacket packet) {
            outtake.spin_flywheel(0, 10);
            telemetry.addData("Flywheel: Status", "Stopped");
            return false;
        }
    }

    public class transferUp implements Action {
        @Override
        public boolean run(TelemetryPacket packet) {
            outtake.transferUp();
            telemetry.addData("Transfer: Status", "Up");
            return false;
        }
    }

    public class transferOff implements Action {
        @Override
        public boolean run(TelemetryPacket packet) {
            outtake.transferDown();
            telemetry.addData("Transfer: Status", "Off");
            return false;
        }
    }

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
            return !pixelDetected;
        }
    }

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

    public class SetIntakePower implements Action {
        private final double power;

        public SetIntakePower(double power) {
            this.power = power;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            intakeSystem.setPower(power);
            telemetry.addData("Intake: Power", power);
            return false;
        }
    }

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

            return !complete;
        }
    }

    public class startspindexer implements Action {
        @Override
        public boolean run(TelemetryPacket packet) {
            spindexerServo.setPower(0.7);
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

    public class SpinToIntake implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!initialized) {
                spindexer.initSpin();
                initialized = true;
            }

            boolean complete = spindexer.spinToIntake();
            return pauseSpindexer;
        }
    }

    public class ToggleSpindexer implements Action {
        private boolean onOff;

        public ToggleSpindexer(boolean onOff) {
            this.onOff = onOff;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            pauseSpindexer = onOff;
            return false;
        }
    }
}
