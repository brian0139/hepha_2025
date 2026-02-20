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

import org.firstinspires.ftc.teamcode.Alvin.intake;
import org.firstinspires.ftc.teamcode.Brian.spindexerColor;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.opModeDataTransfer;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtakeV3FittedAutolaunch;

@Autonomous(name = "farred")
public class AutonRedPath_Far_FarShoot extends LinearOpMode {
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
    Pose2d beginPose = new Pose2d(57.5, -43.5, Math.toRadians(-54)); // Far side starting position
    MecanumDrive drive = null;
    NormalizedColorSensor intakeSensor;
    FtcDashboard dashboard;
    TelemetryPacket packet;
    boolean pauseSpindexer = false;

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

        // Initialize outtake for FAR side (third parameter is false for far side)
        outtake = new outtakeV3FittedAutolaunch(hardwareMap, "Red", false, drive);
        outtake.setPipeLine(0);
        intakeSystem = new intake(hardwareMap, "intake", "intakeSensor");
        spindexer = new spindexerColor(spindexerServo, intakeMotor, hardwareMap);

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        // Far side shooting position - near the far white triangle
        final Vector2d shootingPos = new Vector2d(34, -23);
        final double shootingAngle = Math.toRadians(-40);

        // Intake path parameters for far side
        final double intakeFinishy = -50;
        final double intakeStarty = -13;
        final double waitTime = 1;
        final double shootTime = 1.5;

        // Row positions for far side (mirrored from close side)
        final double row1XPos = 9;
        final double row2XPos = -16;
        final double row3XPos = -38;

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize hood
        Actions.runBlocking(drive.actionBuilder(beginPose)
                .stopAndAdd(new initHood())
                .stopAndAdd(new SetHoodAngle(45.2))
                .build());

        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            // Sequence 0: Shoot preloaded 3 balls
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            // Start flywheel for preloaded balls
                            .stopAndAdd(new SpinFlywheel(1600, 50))
                            .strafeToLinearHeading(shootingPos, shootingAngle)
                            // Shooting Sequence 0
                            .stopAndAdd(new TurretAutoAimUntilAligned())
                            .stopAndAdd(new transferUp())
                            .stopAndAdd(new RunIntake())
                            .stopAndAdd(new startspindexer())
                            .waitSeconds(shootTime)
                            // Stop Sequence 0
                            .stopAndAdd(new StopFlywheel())
                            .stopAndAdd(new transferOff())
                            .stopAndAdd(new stopspindexer())
                            .stopAndAdd(new StopIntake())
                            .stopAndAdd(new ToggleSpindexer(false))
                            .build());

            // First intake (3 balls from row 1)
            Actions.runBlocking(new ParallelAction(
                    drive.actionBuilder(drive.localizer.getPose())
                            // Start Intake Code 1
                            .strafeToLinearHeading(new Vector2d(row1XPos, intakeStarty), Math.toRadians(-80))
                            .stopAndAdd(new RunIntake())
                            .strafeTo(new Vector2d(row1XPos, intakeFinishy - 4))
                            .stopAndAdd(new ToggleSpindexer(true))
                            .build(),
                    new SpinToIntake()));

            // After first intake - shoot 3 balls
            Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                    .stopAndAdd(new ToggleSpindexer(false))
                    // Stop Intake 1
                    .waitSeconds(waitTime)
                    .stopAndAdd(new StopIntake())
                    .stopAndAdd(new stopspindexer())
                    // Start Flywheel 1
                    .stopAndAdd(new SetHoodEncoder(8020))
                    .stopAndAdd(new SpinFlywheel(1833, 50))
                    .strafeToLinearHeading(new Vector2d(row1XPos, intakeStarty + 10), shootingAngle)
                    // Shoot Sequence 1
                    .stopAndAdd(new TurretAutoAimUntilAligned())
                    .stopAndAdd(new transferUp())
                    .stopAndAdd(new RunIntake())
                    .stopAndAdd(new startspindexer())
                    .waitSeconds(shootTime)
                    // Stop Sequence 1
                    .stopAndAdd(new StopFlywheel())
                    .stopAndAdd(new transferOff())
                    .stopAndAdd(new stopspindexer())
                    .stopAndAdd(new StopIntake())
                    .stopAndAdd(new SetIntakePower(-1))
                    .build());

            // Second intake (3 balls from row 2)
            Actions.runBlocking(new ParallelAction(
                    drive.actionBuilder(drive.localizer.getPose())
                            // Start Intake 2
                            .strafeToLinearHeading(new Vector2d(row2XPos, intakeStarty + 5), Math.toRadians(90))
                            .stopAndAdd(new StopIntake())
                            .stopAndAdd(new RunIntake())
                            .strafeTo(new Vector2d(row2XPos, intakeFinishy - 5))
                            .stopAndAdd(new ToggleSpindexer(true))
                            .build(),
                    new SpinToIntake()));

            // After second intake - shoot 3 balls
            Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                    .stopAndAdd(new ToggleSpindexer(false))
                    // Stop Intake 2
                    .waitSeconds(waitTime)
                    .stopAndAdd(new StopIntake())
                    .stopAndAdd(new stopspindexer())
                    // Start Flywheel 2
                    .stopAndAdd(new SpinFlywheel(1833, 50))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(new Vector2d(row1XPos, intakeStarty + 10), shootingAngle),
                            shootingAngle - Math.toRadians(90))
                    // Shoot Sequence 2
                    .stopAndAdd(new TurretAutoAimUntilAligned())
                    .stopAndAdd(new transferUp())
                    .stopAndAdd(new RunIntake())
                    .stopAndAdd(new startspindexer())
                    .waitSeconds(shootTime)
                    // Stop Sequence 2
                    .stopAndAdd(new StopFlywheel())
                    .stopAndAdd(new StopIntake())
                    .stopAndAdd(new transferOff())
                    .stopAndAdd(new stopspindexer())
                    .stopAndAdd(new SetIntakePower(-1))
                    .build());

            // Third intake (last 3 balls from row 3 for total of 9)
            Actions.runBlocking(new ParallelAction(
                    drive.actionBuilder(drive.localizer.getPose())
                            // Start Intake 3
                            .strafeToLinearHeading(new Vector2d(row3XPos, intakeStarty + 10), Math.toRadians(90))
                            .stopAndAdd(new StopIntake())
                            .stopAndAdd(new RunIntake())
                            .strafeTo(new Vector2d(row3XPos, intakeFinishy - 5))
                            .stopAndAdd(new ToggleSpindexer(true))
                            .build(),
                    new SpinToIntake()));

            // After third intake - shoot final 3 balls (total 9)
            Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                    .stopAndAdd(new ToggleSpindexer(false))
                    // Stop Intake 3
                    .waitSeconds(waitTime)
                    .stopAndAdd(new StopIntake())
                    .stopAndAdd(new stopspindexer())
                    // Start Flywheel 3
                    .stopAndAdd(new SetIntakePower(-1))
                    .stopAndAdd(new StopIntake())
                    .stopAndAdd(new SpinFlywheel(1833, 50))
                    .strafeToLinearHeading(new Vector2d(row1XPos, intakeStarty + 10), shootingAngle)
                    // Shoot Sequence 3
                    .stopAndAdd(new TurretAutoAimUntilAligned())
                    .stopAndAdd(new transferUp())
                    .stopAndAdd(new RunIntake())
                    .stopAndAdd(new startspindexer())
                    .stopAndAdd(new TurretAutoAimUntilAligned())
                    .waitSeconds(shootTime)
                    // Stop Sequence 3 and park
                    .stopAndAdd(new StopFlywheel())
                    .stopAndAdd(new transferOff())
                    .stopAndAdd(new stopspindexer())
                    .stopAndAdd(new StopIntake())
                    // Park near backstage
                    .strafeToLinearHeading(new Vector2d(48, -60), Math.toRadians(-90))
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