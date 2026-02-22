package org.firstinspires.ftc.teamcode.Aaron.autonPath;

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

@Autonomous
public class RoadrunnerTest extends LinearOpMode {
    outtakeV3FittedAutolaunch outtake;
    intake intakeSystem;
    spindexerColor spindexer;
    CRServo spindexerServo=null;
    ElapsedTime timer=new ElapsedTime();
    DcMotor intakeMotor=null;
    DcMotorEx transfer=null;
    DcMotorEx flywheel=null;
    DcMotorEx flywheelR=null;
    CRServo hood=null;
    Pose2d beginPose=new Pose2d(37, -33, Math.toRadians(90));
    MecanumDrive drive=null;
    NormalizedColorSensor intakeSensor;
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;
    boolean pauseSpindexer=false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize subsystems
        spindexerServo=hardwareMap.get(CRServo.class,"spindexerServo");
        intakeMotor=hardwareMap.dcMotor.get("intake");
        transfer=(DcMotorEx) hardwareMap.dcMotor.get("par1");
        flywheel=(DcMotorEx) hardwareMap.dcMotor.get("flywheel");
        flywheelR=(DcMotorEx) hardwareMap.dcMotor.get("flywheelR");
        hood=hardwareMap.crservo.get("hoodServo");
        drive=new MecanumDrive(hardwareMap,beginPose);
        intakeSensor=hardwareMap.get(NormalizedColorSensor.class,"intakeSensor");
        outtake = new outtakeV3FittedAutolaunch(hardwareMap,"Red",true,drive);
        outtake.setPipeLine(0);
        intakeSystem = new intake(hardwareMap,"intake","intakeSensor");
        spindexer=new spindexerColor(spindexerServo,intakeMotor,hardwareMap);

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        final Vector2d shootingPos=new Vector2d(-34,23);
        final double shootingAngle=Math.toRadians(140);
        final double intakeFinishy = 50;
        final double intakeStarty=13;
        final double waitTime=1;
        final double shootTime=1.5;
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

        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) return;
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
//                            .setVelocityConstraint()
                            .strafeToSplineHeading(new Vector2d(37, 33), Math.toRadians(180))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(-37, 33), Math.toRadians(270))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(-37, -33), Math.toRadians(0))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(37, -33), Math.toRadians(90))
                            .waitSeconds(0.5)

                            .strafeToSplineHeading(new Vector2d(37, 33), Math.toRadians(180))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(-37, 33), Math.toRadians(270))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(-37, -33), Math.toRadians(0))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(37, -33), Math.toRadians(90))
                            .waitSeconds(0.5)

                            .strafeToSplineHeading(new Vector2d(37, 33), Math.toRadians(180))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(-37, 33), Math.toRadians(270))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(-37, -33), Math.toRadians(0))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(37, -33), Math.toRadians(90))
                            .waitSeconds(0.5)

                            .strafeToSplineHeading(new Vector2d(37, 33), Math.toRadians(180))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(-37, 33), Math.toRadians(270))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(-37, -33), Math.toRadians(0))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(37, -33), Math.toRadians(90))
                            .waitSeconds(0.5)

                            .strafeToSplineHeading(new Vector2d(37, 33), Math.toRadians(180))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(-37, 33), Math.toRadians(270))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(-37, -33), Math.toRadians(0))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(37, -33), Math.toRadians(90))
                            .waitSeconds(0.5)

                            .strafeToSplineHeading(new Vector2d(37, 33), Math.toRadians(180))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(-37, 33), Math.toRadians(270))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(-37, -33), Math.toRadians(0))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(37, -33), Math.toRadians(90))
                            .waitSeconds(0.5)

                            .strafeToSplineHeading(new Vector2d(37, 33), Math.toRadians(180))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(-37, 33), Math.toRadians(270))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(-37, -33), Math.toRadians(0))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(37, -33), Math.toRadians(90))
                            .waitSeconds(0.5)

                            .strafeToSplineHeading(new Vector2d(37, 33), Math.toRadians(180))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(-37, 33), Math.toRadians(270))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(-37, -33), Math.toRadians(0))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(37, -33), Math.toRadians(90))
                            .waitSeconds(0.5)
                            .build());
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

    //TODO: Integrate motif recognition into auton and make it have the ability to recognize the motif not the tower
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
        private boolean isComplete = false;
        private final double alignmentThreshold = 2; // degrees, adjust as needed
        ElapsedTime timer=new ElapsedTime();

        public TurretAutoAimUntilAligned() {
            this.timer.reset();
        }

        @Override
        public boolean run(TelemetryPacket telemetryPacket) {
            if (isComplete) return false;
            if (timer.milliseconds()>=3500){
                outtake.turretServo.setPower(0);
                return false;
            }
            if (!initialized){
                initialized=true;
                outtake.turnPID.init();
            }

            boolean hasTarget = outtake.autoturn();

            if (!hasTarget) {
                telemetry.addData("Turret: Status", "No Target");
                // Optionally complete after some attempts or keep trying
                return true;
            }

            // Check if aligned by examining heading error
            double headingError = Math.abs(outtake.apriltag.getYaw());
            if (headingError < alignmentThreshold) {
                outtake.turretServo.setPower(0); // Stop the turret
                isComplete = true;
                telemetry.addData("Turret: Status", "Aligned!");
                return false; // Action complete
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

        public SetHoodEncoder(double angleDegrees) {
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
            boolean atPosition = outtake.setHoodEncoder(targetAngle);

            // Return false when at position (action complete)
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


    /**
     * Spins spindexer to next empty slot for intake
     */
    public class SpinToIntake implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!initialized) {
                spindexer.initSpin();
                initialized = true;
            }

            boolean complete = spindexer.spinToIntake();

            return pauseSpindexer; // Return false when complete
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