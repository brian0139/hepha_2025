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
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Brian.spindexerColor;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.opModeDataTransfer;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtakeV3FittedAutolaunch;

@Autonomous
public class AutonFarRedPath extends LinearOpMode {
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

    // Start: white triangle top-right, facing left into field
    Pose2d beginPose = new Pose2d(57.5, 43.5, Math.toRadians(180));

    MecanumDrive drive = null;
    NormalizedColorSensor intakeSensor;
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;
    boolean pauseSpindexer = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize subsystems
        spindexerServo = hardwareMap.get(CRServo.class, "spindexerServo");
        intakeMotor = hardwareMap.dcMotor.get("intake");
        transfer = (DcMotorEx) hardwareMap.dcMotor.get("par1");
        hood = hardwareMap.crservo.get("hoodServo");
        drive = new MecanumDrive(hardwareMap, beginPose);
        intakeSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeSensor");
        outtake = new outtakeV3FittedAutolaunch(hardwareMap, "Blue", true, drive);
        outtake.setPipeLine(0);
        intakeSystem = new intake(hardwareMap, "intake", "intakeSensor");
        spindexer = new spindexerColor(spindexerServo, intakeMotor, hardwareMap);

        // White triangle is BOTH start and shooting position
        final Vector2d shootingPos  = new Vector2d(57.5, 43.5);
        final double shootingAngle  = Math.toRadians(180);
        final double intakeFinishy  = 56;
        final double intakeStarty   = 46;
        final double waitTime       = 1;
        final double shootTime      = 10;
        final double row3XPos       = 38;   // nearest to white triangle
        final double row2XPos       = 16;   // center column
        final double row1XPos       = -9;   // leftmost column

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        dashboardTelemetry.addData("Hood: Current Angle", 0);
        dashboardTelemetry.addData("Power", 0);
        dashboardTelemetry.addData("Voltage", 0);
        dashboardTelemetry.update();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

//        Actions.runBlocking(drive.actionBuilder(beginPose)
//                .stopAndAdd(new initHood())
//                .stopAndAdd(new SetHoodAngle(45.2))
//                .build());

        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            // ===== PRELOAD: Shoot 3 balls from white triangle (robot starts here) =====
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            // START POSITION IS THE WHITE TRIANGLE — shoot in place
                            .stopAndAdd(new SpinFlywheel(1600, 50))
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

            // ===== FIRST INTAKE: Row 3 (x=38, nearest cluster to white triangle) =====
            Actions.runBlocking(new ParallelAction(
                    drive.actionBuilder(drive.localizer.getPose())
                            // Start Intake Code 1
                            .strafeToLinearHeading(new Vector2d(row3XPos, intakeStarty), Math.toRadians(90))
                            .stopAndAdd(new RunIntake())
                            .strafeTo(new Vector2d(row3XPos, intakeFinishy))
                            .stopAndAdd(new ToggleSpindexer(true))
                            .build(),
                    new SpinToIntake()));

            // ===== AFTER FIRST INTAKE: Return to white triangle and shoot =====
            Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                    .stopAndAdd(new ToggleSpindexer(false))
                    // Stop Intake 1
                    .waitSeconds(waitTime)
                    .stopAndAdd(new StopIntake())
                    .stopAndAdd(new stopspindexer())
                    // Start Flywheel 1
                    .stopAndAdd(new SetHoodEncoder(8020))
                    .stopAndAdd(new SpinFlywheel(1833, 50))
                    .strafeToLinearHeading(shootingPos, shootingAngle)
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

            // ===== SECOND INTAKE: Row 2 (x=16, center column) =====
            Actions.runBlocking(new ParallelAction(
                    drive.actionBuilder(drive.localizer.getPose())
                            // Start Intake 2
                            .strafeToLinearHeading(new Vector2d(row2XPos, intakeStarty), Math.toRadians(90))
                            .stopAndAdd(new StopIntake())
                            .stopAndAdd(new RunIntake())
                            .strafeTo(new Vector2d(row2XPos, intakeFinishy))
                            .stopAndAdd(new ToggleSpindexer(true))
                            .build(),
                    new SpinToIntake()));

            // ===== AFTER SECOND INTAKE: Return to white triangle and shoot =====
            Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                    .stopAndAdd(new ToggleSpindexer(false))
                    // Stop Intake 2
                    .waitSeconds(waitTime)
                    .stopAndAdd(new StopIntake())
                    .stopAndAdd(new stopspindexer())
                    // Start Flywheel 2
                    .stopAndAdd(new SpinFlywheel(1833, 50))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(shootingPos, shootingAngle), shootingAngle + Math.toRadians(90))
                    // Shoot Sequence 2
                    .stopAndAdd(new TurretAutoAimUntilAligned())
                    .stopAndAdd(new transferUp())
                    .stopAndAdd(new RunIntake())
                    .stopAndAdd(new startspindexer())
                    .waitSeconds(shootTime)
                    // Stop Sequence 2
                    .stopAndAdd(new StopFlywheel())
                    .stopAndAdd(new transferOff())
                    .stopAndAdd(new stopspindexer())
                    .stopAndAdd(new StopIntake())
                    .stopAndAdd(new SetIntakePower(-1))
                    .build());

            // ===== THIRD INTAKE: Row 1 (x=-9, leftmost column) =====
            Actions.runBlocking(new ParallelAction(
                    drive.actionBuilder(drive.localizer.getPose())
                            // Start Intake 3
                            .strafeToLinearHeading(new Vector2d(row1XPos, intakeStarty), Math.toRadians(90))
                            .stopAndAdd(new StopIntake())
                            .stopAndAdd(new RunIntake())
                            .strafeTo(new Vector2d(row1XPos, intakeFinishy))
                            .stopAndAdd(new ToggleSpindexer(true))
                            .build(),
                    new SpinToIntake()));

            // ===== AFTER THIRD INTAKE: Return to white triangle and shoot =====
            Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                    .stopAndAdd(new ToggleSpindexer(false))
                    // Stop Intake 3
                    .waitSeconds(waitTime)
                    .stopAndAdd(new StopIntake())
                    .stopAndAdd(new stopspindexer())
                    // Start Flywheel 3
                    .stopAndAdd(new SpinFlywheel(1833, 50))
                    .strafeToLinearHeading(shootingPos, shootingAngle)
                    // Shoot Sequence 3
                    .stopAndAdd(new TurretAutoAimUntilAligned())
                    .stopAndAdd(new transferUp())
                    .stopAndAdd(new RunIntake())
                    .stopAndAdd(new startspindexer())
                    .stopAndAdd(new TurretAutoAimUntilAligned())
                    .waitSeconds(shootTime)
                    // Stop Sequence 3
                    .stopAndAdd(new StopFlywheel())
                    .stopAndAdd(new transferOff())
                    .stopAndAdd(new stopspindexer())
                    .stopAndAdd(new StopIntake())
                    .build());

            break;
        }
        opModeDataTransfer.currentPose = drive.localizer.getPose();
    }

    // ==================== TURRET ACTIONS ====================
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

    // ==================== HOOD ACTIONS ====================
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
            boolean atPosition = outtake.setHoodEncoder(targetAngle);
            telemetry.addData("Hood: Target", (66.81 - targetAngle) / outtake.servoDegPerRot * outtake.ticksPerRevHood);
            telemetry.addData("Hood: Current Angle", outtake.hoodEncoder.getCurrentPosition());
            telemetry.addData("Power", outtake.hoodPID.power);
            dashboardTelemetry.addData("Hood: Target", (66.81 - targetAngle) / outtake.servoDegPerRot * outtake.ticksPerRevHood);
            dashboardTelemetry.addData("Hood: Current Angle", outtake.hoodEncoder.getCurrentPosition());
            dashboardTelemetry.addData("Power", outtake.hoodPID.power);
            dashboardTelemetry.update();
            telemetry.addData("Hood: At Position", atPosition);
            telemetry.update();
            return !atPosition;
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
            return !atPosition;
        }
    }

    // ==================== FLYWHEEL ACTIONS ====================
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

    // ==================== TRANSFER ACTIONS ====================
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

    // ==================== INTAKE ACTIONS ====================
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

    // ==================== SPINDEXER ACTIONS ====================
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
            spindexer.spinToIntake();
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