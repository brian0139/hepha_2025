package org.firstinspires.ftc.teamcode.Alvin;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Brian.spindexerColor;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtakeV3FittedAutolaunch;

@Autonomous
public class AutonRedPath_Far_FarShoot extends LinearOpMode {
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
    AnalogInput hoodSensor=null;
    Pose2d beginPose=new Pose2d(60.5, 9, Math.toRadians(180));
    MecanumDrive drive=null;
    NormalizedColorSensor intakeSensor;
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize subsystems
        outtake = new outtakeV3FittedAutolaunch(hardwareMap,"Red",true,drive);
        outtake.setPipeLine(0);
        intakeSystem = new intake(hardwareMap,"intake","intakeSensor");
        spindexerServo=hardwareMap.get(CRServo.class,"spindexerServo");
        spindexer=new spindexerColor(spindexerServo,intakeMotor,hardwareMap);
        intakeMotor=hardwareMap.dcMotor.get("intake");
        transfer=(DcMotorEx) hardwareMap.dcMotor.get("par1");
        flywheel=(DcMotorEx) hardwareMap.dcMotor.get("flywheel");
        flywheelR=(DcMotorEx) hardwareMap.dcMotor.get("flywheelR");
        hood=hardwareMap.crservo.get("hoodServo");
        hoodSensor=hardwareMap.get(AnalogInput.class,"hoodAnalog");
        drive=new MecanumDrive(hardwareMap,beginPose);
        intakeSensor=hardwareMap.get(NormalizedColorSensor.class,"intakeSensor");

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        final Vector2d shootingPos=new Vector2d(62,10);
        final double shootingAngle=Math.toRadians(180);
        final double intakeFinishy=36;
        final double intakeStarty=13;
        final double waitTime=1.5;
        final double shootTime=3;
        final double row1XPos=7;
        final double row2XPos=17;
        final double row3XPos=38;
        final double ballfallPos=59;
        final double flywheelSpeed=2208;

        // Straight intake angle - pointing straight down the field
        final double straightIntakeAngle=Math.toRadians(270);

        dashboard=FtcDashboard.getInstance();
        dashboardTelemetry=dashboard.getTelemetry();
        dashboardTelemetry.addData("Hood: Current Angle",0);
        dashboardTelemetry.addData("Power",0);
        dashboardTelemetry.addData("Voltage",0);
        dashboardTelemetry.update();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        drive.actionBuilder(beginPose)
            .stopAndAdd(new initHood())
            .stopAndAdd(new SetHoodAngle(3053.58100770156));

        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) return;
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            // ========== INITIALIZATION ==========
                            // Starting Position: (63, 9) facing 180°
                            .stopAndAdd(new SpinFlywheel(flywheelSpeed,20))
                            .waitSeconds(waitTime)

                            // ========== SHOOTING SEQUENCE 0 (PRELOAD) ==========
                            // Shoot from starting position
                            .stopAndAdd(new transferUp())
                            .stopAndAdd(new RunIntake())
                            .stopAndAdd(new TurretAutoAimUntilAligned())
                            .stopAndAdd(new startspindexer())
                            .waitSeconds(shootTime)
                            .stopAndAdd(new StopFlywheel())
                            .stopAndAdd(new transferOff())
                            .stopAndAdd(new stopspindexer())
                            .stopAndAdd(new StopIntake())

                            //Move out of way
                            .strafeTo(new Vector2d(60.5,35))
                            .waitSeconds(0.5)
                            .stopAndAdd(new initHood())
                            .stopAndAdd(new SetHoodAngle(50))
//
                            // ========== INTAKE CYCLE 1 (ROW 3 - CLOSEST) ==========
//                            // Move to Row 3: (38, 3) facing 270°
//                            .strafeToLinearHeading(new Vector2d(row3XPos, intakeStarty-10), straightIntakeAngle)
//                            .stopAndAdd(new RunIntake())
//                            .stopAndAdd(new startspindexer())
//
//                            // Drive STRAIGHT forward from y=3 to y=39
//                            .lineToY(intakeFinishy+3)
//                            .waitSeconds(waitTime)
//                            .stopAndAdd(new StopIntake())
//                            .stopAndAdd(new stopspindexer())
//
//                            // Back up STRAIGHT to y=3
//                            .lineToY(intakeStarty-10)
//
//                            // Rotate to shooting angle 180° and shoot
//                            .stopAndAdd(new SpinFlywheel(flywheelSpeed,2208))
//                            .turnTo(shootingAngle)
//                            .stopAndAdd(new transferUp())
//                            .stopAndAdd(new RunIntake())
//                            .stopAndAdd(new TurretAutoAimUntilAligned())
//                            .stopAndAdd(new startspindexer())
//                            .stopAndAdd(new TurretAutoAimUntilAligned())
//                            .waitSeconds(shootTime)
//                            .stopAndAdd(new StopFlywheel())
//                            .stopAndAdd(new transferOff())
//                            .stopAndAdd(new stopspindexer())
//                            .stopAndAdd(new StopIntake())
//
//                            // ========== INTAKE CYCLE 2 (ROW 2) ==========
//                            // Move to Row 2: (17, 6) facing 270°
//                            .strafeToLinearHeading(new Vector2d(row2XPos, intakeStarty-7), straightIntakeAngle)
//                            .stopAndAdd(new RunIntake())
//                            .stopAndAdd(new startspindexer())
//
//                            // Drive STRAIGHT forward from y=6 to y=39
//                            .lineToY(intakeFinishy+3)
//                            .waitSeconds(waitTime)
//                            .stopAndAdd(new StopIntake())
//                            .stopAndAdd(new stopspindexer())
//
//                            // Back up STRAIGHT to y=6
//                            .lineToY(intakeStarty-7)
//
//                            // Rotate to shooting angle 180° and shoot
//                            .stopAndAdd(new SpinFlywheel(flywheelSpeed,2208))
//                            .turnTo(shootingAngle)
//                            .stopAndAdd(new transferUp())
//                            .stopAndAdd(new RunIntake())
//                            .stopAndAdd(new TurretAutoAimUntilAligned())
//                            .stopAndAdd(new startspindexer())
//                            .stopAndAdd(new TurretAutoAimUntilAligned())
//                            .waitSeconds(shootTime)
//                            .stopAndAdd(new StopFlywheel())
//                            .stopAndAdd(new transferOff())
//                            .stopAndAdd(new stopspindexer())
//                            .stopAndAdd(new StopIntake())
//
//                            // ========== INTAKE CYCLE 3 (BALL FALL ZONE) ==========
//                            // Move to ball fall position: (59, 3) facing 270°
//                            .strafeToLinearHeading(new Vector2d(ballfallPos, intakeStarty-10), straightIntakeAngle)
//                            .stopAndAdd(new RunIntake())
//                            .stopAndAdd(new startspindexer())
//
//                            // Drive STRAIGHT forward from y=3 to y=39
//                            .lineToY(intakeFinishy+3)
//                            .waitSeconds(waitTime)
//                            .stopAndAdd(new StopIntake())
//                            .stopAndAdd(new stopspindexer())
//
//                            // Back up STRAIGHT and move to shooting position
//                            .lineToY(intakeStarty-10)
//
//                            // Move to better shooting position if needed, then rotate and shoot
//                            .strafeToLinearHeading(new Vector2d(row3XPos, intakeStarty-10), straightIntakeAngle)
//                            .stopAndAdd(new SpinFlywheel(flywheelSpeed,2208))
//                            .turnTo(shootingAngle)
//                            .stopAndAdd(new transferUp())
//                            .stopAndAdd(new RunIntake())
//                            .stopAndAdd(new TurretAutoAimUntilAligned())
//                            .stopAndAdd(new startspindexer())
//                            .stopAndAdd(new TurretAutoAimUntilAligned())
//                            .waitSeconds(shootTime)
//                            .stopAndAdd(new StopFlywheel())
//                            .stopAndAdd(new transferOff())
//                            .stopAndAdd(new stopspindexer())
//                            .stopAndAdd(new StopIntake())

                            .build());
            break;
        }
        telemetry.addData("Status", "All Tests Complete");
        telemetry.update();
    }

    private void runAction(Action action, long timeoutMs) {
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .stopAndAdd(action)
                        .build()
        );
    }

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
            if (timer.milliseconds()>=1000){
                outtake.turretServo.setPower(0);
                return false;
            }
            if (!initialized){
                initialized=true;
                outtake.turretEpsilon=1.5;
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
            return false;
        }
    }

    public class initHood implements Action{
        @Override
        public boolean run(TelemetryPacket packet){
            outtake.initHoodAngleBlocking();
            return false;
        }
    }

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

            return !atSpeed;
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
            spindexerServo.setPower(0.35);
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

            boolean complete = spindexer.spinToIntake();

            telemetry.addData("Spindexer: Motif Index", motifIndex);
            telemetry.addData("Spindexer: Status", complete ? "Complete" : "Spinning");

            return !complete;
        }
    }
}