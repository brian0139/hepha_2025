package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtakeV2;

@TeleOp

public class fullTest extends LinearOpMode{
    // drivetrain wheel motor declaration
    private DcMotor leftFront=null;
    private DcMotor leftBack=null;
    private DcMotor rightFront=null;
    private DcMotor rightBack=null;
    //other motors
    private DcMotorEx flywheelR =null;
    private DcMotorEx flywheel = null;
    private DcMotorEx intake=null;
    private DcMotor transfer=null;
    //servos
    private CRServo hoodServo=null;
    private CRServo spindexer=null;
    private CRServo turret=null;
    outtakeV2 outtake=null;
    //sensitivity(& other configs)
    double flywheelSensitivity=10;
    double hoodspeed=0.5;
    //FTC dashboard telemetry
    FtcDashboard dashboard=null;
    Telemetry dashboardTelemetry=null;
    //vars
    int flywheelspeed=2000;
    int targetspeed=0;
    boolean flywheelToggle=false;
    boolean transferToggle=false;
    double spindexerpos=0.75;
    double spindexerDialation=0.01;
    double epsilon=0.01;
    double[] spindexerpositions = {0,0.75};
    int spindexerAutoPos=1;
    boolean pasty=false;
    //button state storage
    Gamepad previousgamepad1 = new Gamepad();

    //main loop
    @Override
    public void runOpMode() {
        //initiate drivetrain motors
        leftFront   = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack    = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront   = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack   = hardwareMap.get(DcMotor.class, "rightBack");
        //Reverse motor directions where needed
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //other motors
        flywheelR= hardwareMap.get(DcMotorEx.class,"flywheelR");
        flywheelR.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel=hardwareMap.get(DcMotorEx.class,"flywheel");
        intake=(DcMotorEx) hardwareMap.get(DcMotor.class,"intake");
        //servos
        hoodServo=hardwareMap.get(CRServo.class,"hoodServo");
        spindexer=hardwareMap.get(CRServo.class,"spindexerServo");
        transfer=hardwareMap.get(DcMotor.class,"par1");
        turret=hardwareMap.get(CRServo.class,"turretServo");
        outtake=new outtakeV2(hardwareMap,flywheel,flywheelR,"Red",leftFront,rightFront,leftBack,rightBack,hoodServo,transfer,false);
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        //telemetry message to signify robot waiting
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        //wait for driver to press play
        waitForStart();
        //repeat until opmode ends
        while (opModeIsActive()){
            //flywheel
            if (gamepad1.dpad_up){
                flywheelspeed+=flywheelSensitivity;
            }
            else if (gamepad1.dpad_down && flywheelspeed-flywheelSensitivity>=0){
                flywheelspeed-=flywheelSensitivity;
            }
            else if (gamepad1.dpad_down && flywheelspeed-flywheelSensitivity<0){
                flywheelspeed=0;
            }
            //toggle
            if (gamepad1.yWasPressed()){
                flywheelToggle=!flywheelToggle;
                if (flywheelToggle) {
                    targetspeed=flywheelspeed;
                    flywheelR.setVelocity(targetspeed);
                    flywheel.setVelocity(targetspeed);
                } else{
                    targetspeed=0;
                    flywheelR.setVelocity(0);
                    flywheel.setVelocity(0);
                }
            }
            telemetry.addLine("Flywheel Speed:"+flywheelspeed+" encoder ticks/s, "+flywheelspeed*60/28+" RPM");
            telemetry.addLine("Flywheel Speed:"+targetspeed+" encoder ticks/s, "+targetspeed*60/28+" RPM");
            telemetry.addLine("Flywheel Speed:"+flywheelR.getVelocity()+" encoder ticks/s, "+flywheelR.getVelocity()*60/28+" RPM");
            //spindexer
//            if (spindexerpos-gamepad1.left_stick_x*spindexerDialation>=0 && spindexerpos-gamepad1.left_stick_x*spindexerDialation<=0.75){
//                spindexerpos-=gamepad1.left_stick_x*spindexerDialation;
//            }
//            else if (spindexerpos-gamepad1.left_stick_x*spindexerDialation<0){
//                spindexerpos=0;
//            }
//            else if (spindexerpos-gamepad1.left_stick_x*spindexerDialation>0.75){
//                spindexerpos=0.75;
//            }
            if (gamepad1.right_bumper){
                spindexer.setPower(1);
            }else if (gamepad1.left_bumper){
                spindexer.setPower(-1);
            }else{
                spindexer.setPower(0);
            }
//            if ((spindexer.getPosition()<=spindexerpositions[0]+epsilon) && (spindexer.getPosition()>=spindexerpositions[0]-epsilon) && spindexerAutoPos%2==0){
//                spindexerAutoPos++;
//                spindexerpos=spindexerpositions[spindexerAutoPos%2];
//            }
            //update gamepad+telemetry
            previousgamepad1.copy(gamepad1);
            //transfer
            if (gamepad1.xWasPressed()){
                transferToggle=!transferToggle;
            }
            if (transferToggle) {
                transfer.setPower(1);
                telemetry.addLine("Transfer Position: Up");
            }else{
                transfer.setPower(0);
                telemetry.addLine("Transfer Position: Stopped");
            }
//            telemetry.addData("transfer Target Speed:",transfer.getPosition());
            //hood
            if (gamepad1.dpad_right){
                hoodServo.setPower(hoodspeed);
            }
            else if (gamepad1.dpad_left){
                hoodServo.setPower(-hoodspeed);
            }
            else{
                hoodServo.setPower(0);
            }


            //intake
            intake.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            turret.setPower(-gamepad1.left_stick_x*0.5);
//            //below is drivetrain
//            // Mecanum drive is controlled with three axes: drive (front-and-back),
//            // strafe (left-and-right), and twist (rotating the whole chassis).
//            //Default:0.7
//            double drive  = -gamepad1.left_stick_y*0.7;
//            //Default:0.5
//            double strafe = -gamepad1.left_stick_x*0.5;
//            double twist  = -gamepad1.right_stick_x*0.5;
//            telemetry.addData("drive: ", drive);
//            telemetry.addData("strafe: ", strafe);
//            telemetry.addData("twist: ", twist);
//
//            double[] speeds = {
//                    (drive - strafe - twist), //forward-left motor
//                    (drive + strafe + twist), //forward-right motor
//                    (drive + strafe - twist), //back-left motor
//                    (drive - strafe + twist)  //back-right motor
//            };
//
//            // Loop through all values in the speeds[] array and find the greatest
//            // *magnitude*.  Not the greatest velocity.
//            double max = Math.abs(speeds[0]);
//            for(int i = 0; i < speeds.length; i++) {
//                if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
//            }
//
//            // If and only if the maximum is outside of the range we want it to be,
//            // normalize all the other speeds based on the given speed value.
//            if (max > 1) {
//                for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
//            }
//
//            // apply the calculated values to the motors.
//            leftFront.setPower(speeds[0]);
//            rightFront.setPower(speeds[1]);
//            leftBack.setPower(speeds[2]);
//            rightBack.setPower(speeds[3]);
            telemetry.update();
//            dashboardTelemetry.update();
        }
    }
}