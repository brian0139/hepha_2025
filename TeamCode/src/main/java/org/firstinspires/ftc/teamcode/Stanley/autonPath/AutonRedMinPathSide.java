//package org.firstinspires.ftc.teamcode.Stanley.autonPath;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.Brian.spindexer;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
//@Autonomous
//public class AutonRedMinPathSide extends LinearOpMode {
//    Servo spindexerServo=null;
//    DcMotor intakeMotor=null;
//    Servo transfer=null;
//    DcMotorEx flywheel=null;
//    CRServo hood=null;
//    //TODO:get values for shooting hood angle and flywheel speed
//    final double hoodAngle=0;
//    final int flywheelSpeed=1000;
//    //classes
//    spindexer spindexerOperator=null;
//    @Override
//    public void runOpMode() throws InterruptedException{
//        spindexerServo=hardwareMap.servo.get("spindexerServo");
//        intakeMotor=hardwareMap.dcMotor.get("intake");
//        flywheel=(DcMotorEx) hardwareMap.dcMotor.get("flywheel");
//        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
//        transfer=hardwareMap.servo.get("transferServo");
//        hood=hardwareMap.crservo.get("hoodServo");
//        spindexerOperator=new spindexer(spindexerServo);
//        Pose2d beginPose=new Pose2d(61, 9, Math.toRadians(180));
//        MecanumDrive drive=new MecanumDrive(hardwareMap,beginPose);
//        final Vector2d shootingPos=new Vector2d(-34,-23);
//        final double shootingAngle=Math.toRadians(225);
//        final double intakeFinishy =-36;
//        final double intakeStarty=-13;
//
//        waitForStart();
//        Actions.runBlocking(
//                drive.actionBuilder(beginPose)
//                    .strafeTo(new Vector2d(58,36)).build());
//    }
//}
