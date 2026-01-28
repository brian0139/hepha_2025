//package org.firstinspires.ftc.teamcode.Stanley.autonPath;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Alvin.intake;
//import org.firstinspires.ftc.teamcode.Brian.spindexerColor;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtakeV2;
//import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtakeV3;
//
//@Autonomous
//public class hoodAutonTest extends LinearOpMode {
//    outtakeV3 outtake;
//    intake intakeSystem;
//    spindexerColor spindexer;
//    CRServo spindexerServo=null;
//    ElapsedTime timer=new ElapsedTime();
//    DcMotor intakeMotor=null;
//    DcMotorEx transfer=null;
//    DcMotorEx flywheel=null;
//    DcMotorEx flywheelR=null;
//    CRServo hood=null;
//    AnalogInput hoodSensor=null;
//    Pose2d beginPose=new Pose2d(-57.5, 43.5, Math.toRadians(360-54));
//    MecanumDrive drive=null;
//    NormalizedColorSensor intakeSensor;
//    FtcDashboard dashboard;
//    Telemetry dashboardTelemetry;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // Initialize subsystems
//        outtake = new outtakeV3(hardwareMap,"Red",true,drive);
//        intakeSystem = new intake(hardwareMap,"intake","intakeSensor");
//        spindexer=new spindexerColor(spindexerServo,intakeMotor,hardwareMap);
//        spindexerServo=hardwareMap.crservo.get("spindexerServo");
//        intakeMotor=hardwareMap.dcMotor.get("intake");
//        transfer=(DcMotorEx) hardwareMap.dcMotor.get("par1");
//        flywheel=(DcMotorEx) hardwareMap.dcMotor.get("flywheel");
//        flywheelR=(DcMotorEx) hardwareMap.dcMotor.get("flywheelR");
//        hood=hardwareMap.crservo.get("hoodServo");
//        hoodSensor=hardwareMap.get(AnalogInput.class,"hoodAnalog");
//        drive=new MecanumDrive(hardwareMap,beginPose);
//        intakeSensor=hardwareMap.get(NormalizedColorSensor.class,"intakeSensor");
//
//        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        final Vector2d shootingPos=new Vector2d(-34,23);
//        final double shootingAngle=Math.toRadians(120);
//        final double intakeFinishy =36;
//        final double intakeStarty=13;
//
//        dashboard=FtcDashboard.getInstance();
//        dashboardTelemetry=dashboard.getTelemetry();
//        dashboardTelemetry.addData("Hood: Current Angle",0);
//        dashboardTelemetry.addData("Power",0);
//        dashboardTelemetry.addData("Voltage",0);
//        dashboardTelemetry.update();
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            if (isStopRequested()) return;
//            Actions.runBlocking(
//                    drive.actionBuilder(beginPose)
//                            .stopAndAdd(new SetHoodAngle(45))
//                            .build());
//            break;
//        }
//        telemetry.addData("Status", "All Tests Complete");
//        telemetry.update();
//    }
//    // ==================== HOOD ACTION ====================
//    /**
//     * Sets hood to specific angle and waits until reached
//     */
//    public class SetHoodAngle implements Action {
//        private final double targetAngle;
//        private boolean started = false;
//
//        public SetHoodAngle(double angleDegrees) {
//            this.targetAngle = angleDegrees;
//        }
//
//        @Override
//        public boolean run(TelemetryPacket packet) {
//            if (!started) {
//                started = true;
//                telemetry.addData("Hood: Target Angle", targetAngle);
//                telemetry.update();
//                outtake.initHoodAngleBlocking();
////                outtake.hoodEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
////                while(outtake.hoodEncoder.getCurrentPosition()>=-3*outtake.ticksPerRevHood){
////                    telemetry.addData("position",outtake.hoodEncoder.getCurrentPosition());
////                    telemetry.update();
////                    outtake.hoodServo.setPower(1);
////                }
////                outtake.hoodServo.setPower(0);
////                outtake.hoodEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//                outtake.hoodPID.init();
//                telemetry.update();
////                outtake.updateHoodAngle();
//            }
//
//            // Update hood position
//            boolean atPosition = outtake.setHood(targetAngle);
//
//            telemetry.addData("Hood: Target",(66.81-targetAngle)/outtake.servoDegPerRot*outtake.ticksPerRevHood);
//            telemetry.addData("Hood: Current Angle", outtake.hoodEncoder.getCurrentPosition());
//            telemetry.addData("Power",outtake.hoodPID.power);
//            dashboardTelemetry.addData("Hood: Target",(66.81-targetAngle)/outtake.servoDegPerRot*outtake.ticksPerRevHood);
//            dashboardTelemetry.addData("Hood: Current Angle", outtake.hoodEncoder.getCurrentPosition());
//            dashboardTelemetry.addData("Power",outtake.hoodPID.power);
//            dashboardTelemetry.update();
//            telemetry.addData("Hood: At Position", atPosition);
//            telemetry.update();
//
//            // Return false when at position (action complete)
//            return true;
//        }
//    }
//}