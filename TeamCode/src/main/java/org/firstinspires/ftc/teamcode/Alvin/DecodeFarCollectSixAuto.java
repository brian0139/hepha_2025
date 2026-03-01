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

    public static double PARK_X = 60.5;
    public static double PARK_Y = -35.0;
    public static double PARK_HEADING_DEG = 180.0;

    public static int PIPELINE = 2;
    public static double SPINDEXER_POWER = 0.7;

    public static double TURRET_ALIGN_THRESHOLD_DEG = 2.0;
    public static double TURRET_TIMEOUT_MS = 3500;

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

        outtake = new outtakeV3FittedAutolaunch(hardwareMap, "Red", false, drive);
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
        runBlockingAbortable(
                drive.actionBuilder(beginPose)
                        .stopAndAdd(new SpinFlywheel(PRELOAD_FLYWHEEL_TPS, FLYWHEEL_TOLERANCE_TPS))
                        .strafeToLinearHeading(shootingPos, shootingAngle)
                        .stopAndAdd(new TurretAutoAimUntilAligned())
                        .stopAndAdd(new transferUp())
                        .stopAndAdd(new RunIntake())
                        .stopAndAdd(new startspindexer())
                        .waitSeconds(SHOOT_TIME_S)
                        .stopAndAdd(new StopFlywheel())
                        .stopAndAdd(new transferOff())
                        .stopAndAdd(new stopspindexer())
                        .stopAndAdd(new StopIntake())
                        .stopAndAdd(new ToggleSpindexer(false))
                        .build());

        // ===== Intake cycle 1 (3 balls) =====
        runBlockingAbortable(new ParallelAction(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(
                                new Vector2d(ROW1_X, INTAKE_START_Y),
                                Math.toRadians(INTAKE_HEADING_DEG))
                        .stopAndAdd(new StopIntake())
                        .stopAndAdd(new RunIntake())
                        .strafeTo(new Vector2d(ROW1_X, INTAKE_FINISH_Y))
                        .stopAndAdd(new ToggleSpindexer(true))
                        .build(),
                new SpinToIntake()));

        // ===== Shoot cycle 1 (total 6 balls) =====
        runBlockingAbortable(
                drive.actionBuilder(drive.localizer.getPose())
                        .stopAndAdd(new ToggleSpindexer(false))
                        .waitSeconds(WAIT_TIME_S)
                        .stopAndAdd(new StopIntake())
                        .stopAndAdd(new stopspindexer())
                        .stopAndAdd(new SetHoodEncoder(CYCLE_HOOD_ENCODER))
                        .stopAndAdd(new SpinFlywheel(CYCLE_FLYWHEEL_TPS, FLYWHEEL_TOLERANCE_TPS))
                        .strafeToLinearHeading(new Vector2d(RETURN_X, RETURN_Y), shootingAngle)
                        .stopAndAdd(new TurretAutoAimUntilAligned())
                        .stopAndAdd(new transferUp())
                        .stopAndAdd(new RunIntake())
                        .stopAndAdd(new startspindexer())
                        .waitSeconds(SHOOT_TIME_S)
                        .stopAndAdd(new StopFlywheel())
                        .stopAndAdd(new transferOff())
                        .stopAndAdd(new stopspindexer())
                        .stopAndAdd(new StopIntake())
                        .build());

        // ===== Park =====
        runBlockingAbortable(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(new Vector2d(PARK_X, PARK_Y), Math.toRadians(PARK_HEADING_DEG))
                        .build());

        opModeDataTransfer.currentPose = drive.localizer.getPose();
    }

    // ===== Actions =====
    public class TurretAutoAimUntilAligned implements Action {
        private boolean initialized = false;
        private boolean isComplete = false;
        ElapsedTime timer = new ElapsedTime();

        public TurretAutoAimUntilAligned() {
            this.timer.reset();
        }

        @Override
        public boolean run(TelemetryPacket telemetryPacket) {
            if (isComplete) return false;
            if (timer.milliseconds() >= TURRET_TIMEOUT_MS) {
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
            if (headingError < TURRET_ALIGN_THRESHOLD_DEG) {
                outtake.turretServo.setPower(0);
                isComplete = true;
                telemetry.addData("Turret: Status", "Aligned!");
                return false;
            }

            telemetry.addData("Turret: Status", "Aligning");
            return true;
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
        private boolean started = false;

        public SetHoodEncoder(double targetEncoder) {
            this.targetEncoder = targetEncoder;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!started) {
                started = true;
                outtake.hoodPID.init();
            }

            boolean atPosition = outtake.setHoodEncoder(targetEncoder);
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
        @Override
        public boolean run(TelemetryPacket packet) {
            spindexerServo.setPower(SPINDEXER_POWER);
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
            return !complete && !pauseSpindexer;
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
