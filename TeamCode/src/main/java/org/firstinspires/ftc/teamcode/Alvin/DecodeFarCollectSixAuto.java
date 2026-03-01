package org.firstinspires.ftc.teamcode.Alvin;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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

import org.firstinspires.ftc.teamcode.Brian.spindexerColor;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.opModeDataTransfer;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtakeV3FittedAutolaunch;

@Config
@Autonomous(name = "Decode Far Collect 6", group = "Autonomous")
public class DecodeFarCollectSixAuto extends LinearOpMode {
    // ===== Tunable Parameters (AutonRedPathV2-style) =====
    public static double START_X = 63.0;
    public static double START_Y = 9.0;
    public static double START_HEADING_DEG = 180.0;

    public static double SHOOT_X = 62.0;
    public static double SHOOT_Y = 10.0;
    public static double SHOOT_HEADING_DEG = 180.0;

    // Nearest far-side column from the (63, 9) start.
    public static double ROW1_X = 38.0;
    public static double INTAKE_START_Y = 13.0;
    public static double INTAKE_FINISH_Y = 36.0;
    public static double INTAKE_HEADING_DEG = 270.0;

    public static double RETURN_X = 62.0;
    public static double RETURN_Y = 10.0;

    public static double WAIT_TIME_S = 1.0;
    public static double SHOOT_TIME_S = 1.5;

    public static double PRELOAD_FLYWHEEL_TPS = 1600;
    public static double CYCLE_FLYWHEEL_TPS = 1833;
    public static int FLYWHEEL_TOLERANCE_TPS = 50;

    public static double INIT_HOOD_ANGLE_DEG = 45.2;
    public static double CYCLE_HOOD_ENCODER = 8020;
    public static double PRELOAD_HOOD_ENCODER = 6060;
    public static double PRELOAD_HOOD_EPSILON = 75;
    public static double CYCLE_HOOD_EPSILON = 100;

    public static double PRELOAD_SPINUP_TPS = 9999;
    public static double PRELOAD_TARGET_TPS = 1531;
    public static int PRELOAD_TARGET_TOLERANCE_TPS = 25;
    public static int PRELOAD_PARALLEL_TARGET_TOLERANCE_TPS = 50;
    public static double CYCLE_TARGET_TPS = 1833;
    public static int CYCLE_TARGET_TOLERANCE_TPS = 50;

    public static double PRELOAD_TURRET_EPSILON = 0.15;
    public static double CYCLE_TURRET_EPSILON = 0.4;

    public static double PARK_X = 60.5;
    public static double PARK_Y = -35.0;
    public static double PARK_HEADING_DEG = 180.0;

    public static int PIPELINE = 2;
    public static double SPINDEXER_POWER = 0.7;

    public static double TURRET_ALIGN_THRESHOLD_DEG = 2.0;
    public static double TURRET_TIMEOUT_MS = 3500;
    public static double MOTIF_SCAN_TIMEOUT_MS = 1200;
    public static int DEFAULT_MOTIF_INDEX = 0;

    // ===== Subsystems =====
    outtakeV3FittedAutolaunch outtake;
    intake intakeSystem;
    spindexerColor spindexer;

    CRServo spindexerServo = null;
    DcMotor intakeMotor = null;
    DcMotorEx transfer = null;
    DcMotorEx flywheel = null;
    DcMotorEx flywheelR = null;
    CRServo hood = null;

    MecanumDrive drive = null;
    NormalizedColorSensor intakeSensor;

    FtcDashboard dashboard;

    boolean pauseSpindexer = false;
    int scannedMotifIndex = DEFAULT_MOTIF_INDEX;
    boolean toggleTurret = false;
    boolean turretAligned = false;

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
        Pose2d beginPose = new Pose2d(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
        Vector2d shootingPos = new Vector2d(SHOOT_X, SHOOT_Y);
        double shootingAngle = Math.toRadians(SHOOT_HEADING_DEG);

        // Initialize subsystems
        spindexerServo = hardwareMap.get(CRServo.class, "spindexerServo");
        intakeMotor = hardwareMap.dcMotor.get("intake");
        transfer = (DcMotorEx) hardwareMap.dcMotor.get("par1");
        flywheel = (DcMotorEx) hardwareMap.dcMotor.get("flywheel");
        flywheelR = (DcMotorEx) hardwareMap.dcMotor.get("flywheelR");
        hood = hardwareMap.crservo.get("hoodServo");

        drive = new MecanumDrive(hardwareMap, beginPose);
        intakeSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeSensor");

        outtake = new outtakeV3FittedAutolaunch(hardwareMap, "Red", true, drive);
        outtake.setPipeLine(PIPELINE);
        intakeSystem = new intake(hardwareMap, "intake", "intakeSensor");
        spindexer = new spindexerColor(spindexerServo, intakeMotor, hardwareMap);

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        dashboard = FtcDashboard.getInstance();
        dashboard.getTelemetry().addData("Status", "Initialized");
        dashboard.getTelemetry().update();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runBlockingAbortable(
                drive.actionBuilder(beginPose)
                        .stopAndAdd(new initHood())
                        .stopAndAdd(new SetHoodAngle(INIT_HOOD_ANGLE_DEG))
                        .build());

        waitForStart();
        if (!opModeIsActive() || isStopRequested()) return;

        // ===== Preload shoot (3 balls) =====
        runBlockingAbortable(new ParallelAction(
                drive.actionBuilder(beginPose)
                        .stopAndAdd(new SpinFlywheel(PRELOAD_SPINUP_TPS, FLYWHEEL_TOLERANCE_TPS))
                        .strafeToLinearHeading(shootingPos, shootingAngle)
                        .stopAndAdd(new SetHoodEncoder(PRELOAD_HOOD_ENCODER, PRELOAD_HOOD_EPSILON))
                        .stopAndAdd(new awaitSpinFlywheel(PRELOAD_TARGET_TPS, PRELOAD_TARGET_TOLERANCE_TPS))
                        .stopAndAdd(new TurretAutoAimUntilAligned(
                                PRELOAD_TURRET_EPSILON,
                                PRELOAD_HOOD_EPSILON,
                                PRELOAD_TARGET_TOLERANCE_TPS,
                                TURRET_TIMEOUT_MS))
                        .stopAndAdd(new transferUp())
                        .stopAndAdd(new RunIntake())
                        .stopAndAdd(new rotateSpindexer())
                        .stopAndAdd(new stopspindexer())
                        .stopAndAdd(new transferOff())
                        .stopAndAdd(new StopIntake())
                        .stopAndAdd(new ToggleSpindexer(false))
                        .stopAndAdd(new toggleTurretAutoAim(false))
                        .build(),
                new awaitSpinFlywheel(PRELOAD_TARGET_TPS, PRELOAD_PARALLEL_TARGET_TOLERANCE_TPS),
                new SetHoodEncoder(PRELOAD_HOOD_ENCODER, PRELOAD_HOOD_EPSILON)));

        // ===== Intake cycle 1 (3 balls) =====
        runBlockingAbortable(new ParallelAction(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(
                                new Vector2d(ROW1_X, INTAKE_START_Y),
                                Math.toRadians(INTAKE_HEADING_DEG))
                        .stopAndAdd(new RunIntake())
                        .strafeTo(new Vector2d(ROW1_X, INTAKE_FINISH_Y))
                        .build(),
                new SpinToIntake(-1, 1)));

        // ===== Shoot cycle 1 (total 6 balls) =====
        runBlockingAbortable(new ParallelAction(
                drive.actionBuilder(drive.localizer.getPose())
                        .stopAndAdd(new StopIntake())
                        .stopAndAdd(new stopspindexer())
                        .stopAndAdd(new SetHoodEncoder(CYCLE_HOOD_ENCODER, CYCLE_HOOD_EPSILON))
                        .stopAndAdd(new SpinFlywheel(CYCLE_FLYWHEEL_TPS, FLYWHEEL_TOLERANCE_TPS))
                        .strafeToLinearHeading(new Vector2d(RETURN_X, RETURN_Y), shootingAngle)
                        .stopAndAdd(new awaitSpinFlywheel(CYCLE_TARGET_TPS, CYCLE_TARGET_TOLERANCE_TPS))
                        .stopAndAdd(new TurretAutoAimUntilAligned(
                                CYCLE_TURRET_EPSILON,
                                CYCLE_HOOD_EPSILON,
                                CYCLE_TARGET_TOLERANCE_TPS,
                                TURRET_TIMEOUT_MS))
                        .stopAndAdd(new transferUp())
                        .stopAndAdd(new RunIntake())
                        .stopAndAdd(new rotateSpindexer())
                        .stopAndAdd(new stopspindexer())
                        .stopAndAdd(new transferOff())
                        .stopAndAdd(new StopIntake())
                        .stopAndAdd(new toggleTurretAutoAim(false))
                        .build(),
                new awaitSpinFlywheel(CYCLE_TARGET_TPS, CYCLE_TARGET_TOLERANCE_TPS),
                new SetHoodEncoder(CYCLE_HOOD_ENCODER, CYCLE_HOOD_EPSILON)));

        // ===== Park =====
        runBlockingAbortable(
                drive.actionBuilder(drive.localizer.getPose())
                        .stopAndAdd(new StopFlywheel())
                        .stopAndAdd(new stopspindexer())
                        .strafeToLinearHeading(new Vector2d(PARK_X, PARK_Y), Math.toRadians(PARK_HEADING_DEG))
                        .build());

        opModeDataTransfer.currentPose = drive.localizer.getPose();
    }

    // ===== Actions =====
    public class TurretAutoAimUntilAligned implements Action {
        private boolean initialized = false;
        private final double alignmentThreshold;
        private final double hoodEpsilon;
        private final int flywheelEpsilon;
        private final double timeoutMs;
        ElapsedTime timer = new ElapsedTime();

        public TurretAutoAimUntilAligned() {
            this(TURRET_ALIGN_THRESHOLD_DEG, CYCLE_HOOD_EPSILON, FLYWHEEL_TOLERANCE_TPS, TURRET_TIMEOUT_MS);
        }

        public TurretAutoAimUntilAligned(double epsilon, double hoodEpsilon, int flywheelEpsilon, double timeoutMs) {
            this.alignmentThreshold = epsilon;
            this.hoodEpsilon = hoodEpsilon;
            this.flywheelEpsilon = flywheelEpsilon;
            this.timeoutMs = timeoutMs;
            this.timer.reset();
            toggleTurret = false;
            turretAligned = false;
            outtake.turretEpsilon = epsilon;
        }

        @Override
        public boolean run(TelemetryPacket telemetryPacket) {
            if (timer.milliseconds() >= timeoutMs || toggleTurret) {
                outtake.turretServo.setPower(0);
                outtake.hoodServo.setPower(0);
                turretAligned = false;
                return false;
            }
            if (!initialized) {
                initialized = true;
                outtake.turnPID.init();
                outtake.hoodPID.init();
                outtake.epsilonHood = hoodEpsilon;
            }

            boolean hasTarget = outtake.autoturn();
            if (!hasTarget) {
                telemetry.addData("Turret: Status", "No Target");
                turretAligned = false;
                return true;
            }

            double headingError = Math.abs(outtake.apriltag.getYaw());
            boolean flywheelAtSpeed = outtake.spin_flywheel(CYCLE_TARGET_TPS, flywheelEpsilon);
            if (headingError < alignmentThreshold && flywheelAtSpeed) {
                outtake.turretServo.setPower(0);
                turretAligned = true;
                telemetry.addData("Turret: Status", "Aligned!");
                return false;
            }

            telemetry.addData("Turret: Status", "Aligning");
            turretAligned = false;
            return true;
        }
    }

    /**
     * Scans AprilTag motif first, then spins spindexer to that motif slot.
     * Motif codes are 1..3 from aprilTagV3, converted to 0..2 for spindexer.
     */
    public class ScanThenSpinToMotif implements Action {
        private final ElapsedTime timer = new ElapsedTime();
        private boolean initialized = false;
        private boolean scanComplete = false;
        private int motifIndex = DEFAULT_MOTIF_INDEX;

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
                timer.reset();
            }

            if (!scanComplete) {
                if (outtake.apriltag == null) {
                    motifIndex = DEFAULT_MOTIF_INDEX;
                    scannedMotifIndex = motifIndex;
                    scanComplete = true;
                    spindexer.initSpin();
                } else {
                    outtake.apriltag.scanOnce();
                    int motifCode = outtake.apriltag.getMotifCode();
                    if (motifCode >= 1 && motifCode <= 3) {
                        motifIndex = motifCode - 1;
                        scannedMotifIndex = motifIndex;
                        scanComplete = true;
                        spindexer.initSpin();
                    } else if (timer.milliseconds() >= MOTIF_SCAN_TIMEOUT_MS) {
                        motifIndex = DEFAULT_MOTIF_INDEX;
                        scannedMotifIndex = motifIndex;
                        scanComplete = true;
                        spindexer.initSpin();
                    } else {
                        telemetry.addData("Motif", "Scanning...");
                        return true;
                    }
                }
            }

            boolean complete = spindexer.spinToMotif(motifIndex);
            telemetry.addData("Motif Index", motifIndex);
            telemetry.addData("Spindexer", complete ? "Aligned" : "Spinning");
            return !complete;
        }
    }

    public Action scanThenSpinToMotif() {
        return new ScanThenSpinToMotif();
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
            return !atPosition;
        }
    }

    public class SetHoodEncoder implements Action {
        private final double targetEncoder;
        private final double epsilon;
        private boolean started = false;

        public SetHoodEncoder(double targetEncoder) {
            this(targetEncoder, CYCLE_HOOD_EPSILON);
        }

        public SetHoodEncoder(double targetEncoder, double epsilon) {
            this.targetEncoder = targetEncoder;
            this.epsilon = epsilon;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!started) {
                started = true;
                outtake.hoodPID.init();
                outtake.epsilonHood = epsilon;
            }

            boolean atPosition = outtake.setHoodEncoder(targetEncoder);
            if (atPosition) {
                outtake.hoodServo.setPower(0);
            }
            return !atPosition;
        }
    }

    public class SpinFlywheel implements Action {
        private final double targetSpeed;
        private final int tolerance;

        public SpinFlywheel(double targetSpeedTicksPerSec, int toleranceTicksPerSec) {
            this.targetSpeed = targetSpeedTicksPerSec;
            this.tolerance = toleranceTicksPerSec;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            outtake.spin_flywheel(targetSpeed, tolerance);
            return false;
        }
    }

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
            }
            boolean atSpeed = outtake.spin_flywheel(targetSpeed, tolerance);
            telemetry.addData("Flywheel: Current Speed", outtake.flywheelDriveR.getVelocity());
            telemetry.addData("Flywheel: At Speed", atSpeed);
            return !atSpeed;
        }
    }

    public class StopFlywheel implements Action {
        @Override
        public boolean run(TelemetryPacket packet) {
            outtake.spin_flywheel(0, 10);
            return false;
        }
    }

    public class transferUp implements Action {
        @Override
        public boolean run(TelemetryPacket packet) {
            outtake.transferUp();
            return false;
        }
    }

    public class transferOff implements Action {
        @Override
        public boolean run(TelemetryPacket packet) {
            outtake.transferDown();
            return false;
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

    public class startspindexer implements Action {
        private final double power;

        public startspindexer() {
            this(SPINDEXER_POWER);
        }

        public startspindexer(double power) {
            this.power = power;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            spindexerServo.setPower(power);
            return false;
        }
    }

    public class rotateSpindexer implements Action {
        private int startEncoder;
        private boolean initialized = false;

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!initialized) {
                startEncoder = spindexer.spindexerSensor.getCurrentPosition();
                initialized = true;
            }
            if (Math.abs(spindexer.spindexerSensor.getCurrentPosition() - startEncoder) <= 8192) {
                spindexerServo.setPower(1);
                return true;
            }
            spindexerServo.setPower(0);
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

    public class toggleTurretAutoAim implements Action {
        private final boolean toggle;

        public toggleTurretAutoAim(boolean toggle) {
            this.toggle = toggle;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            toggleTurret = !toggle;
            return false;
        }
    }

    public class SpinToIntake implements Action {
        private boolean initialized = false;
        private final ElapsedTime timer = new ElapsedTime();
        private final double timeoutMs;
        private final double intakePower;

        public SpinToIntake() {
            this(-1, 1);
        }

        public SpinToIntake(double timeoutMs, double intakePower) {
            this.timeoutMs = timeoutMs;
            this.intakePower = intakePower;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!initialized) {
                spindexer.initSpin();
                timer.reset();
                initialized = true;
            }

            if (timeoutMs != -1 && timer.milliseconds() >= timeoutMs) {
                spindexerServo.setPower(0);
                intakeSystem.stop();
                return false;
            }

            intakeSystem.setPower(intakePower);
            boolean complete = spindexer.spinToIntake();
            if (pauseSpindexer || complete) {
                spindexerServo.setPower(0);
                if (complete) {
                    intakeSystem.stop();
                }
                return false;
            }
            return true;
        }
    }

    public class ToggleSpindexer implements Action {
        private final boolean onOff;

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
