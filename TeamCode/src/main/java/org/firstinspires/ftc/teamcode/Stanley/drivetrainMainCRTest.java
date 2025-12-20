package org.firstinspires.ftc.teamcode.Stanley;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class drivetrainMainCRTest extends LinearOpMode{
    // drivetrain wheel motor declaration
    private DcMotor leftFront=null;
    private DcMotor leftBack=null;
    private DcMotor rightFront=null;
    private DcMotor rightBack=null;
    //other motors
    private DcMotorEx flywheel=null;
    private DcMotorEx intake=null;
    private DcMotor transfer=null;
    //servos
    private CRServo hoodServo=null;
    private Servo spindexer=null;
    //sensitivity(& other configs)
    double flywheelSensitivity=10;
    //vars
    int flywheelspeed=2000;
    int targetspeed=0;
    boolean flywheelToggle=false;
    boolean transferToggle=false;
    double spindexerpos=0.75;
    double spindexerDialation=0.01;
    double epsilon=0.05;
    double[] spindexerpositions = {0,0.75};
    int spindexerAutoPos=1;
    boolean pasty=false;
    //button state storage
    Gamepad previousgamepad2 =new Gamepad();
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
        flywheel=(DcMotorEx) hardwareMap.get(DcMotor.class,"flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        intake=(DcMotorEx) hardwareMap.get(DcMotor.class,"intake");
        //servos
        hoodServo=hardwareMap.get(CRServo.class,"hoodServo");
        spindexer=hardwareMap.get(Servo.class,"spindexerServo");
        spindexer.setPosition(spindexerpos);
        transfer=hardwareMap.get(DcMotor.class,"par1");

        //telemetry message to signify robot waiting
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        //wait for driver to press play
        waitForStart();
        //repeat until opmode ends
        while (opModeIsActive()){
            //flywheel
            if (flywheelspeed-gamepad2.right_stick_y*flywheelSensitivity>=0){
                flywheelspeed-=gamepad2.right_stick_y*flywheelSensitivity;
            }
            else{
                flywheelspeed=0;
            }
            //toggle
            if (gamepad2.yWasPressed()){
                flywheelToggle=!flywheelToggle;
                if (flywheelToggle) {
                    targetspeed=flywheelspeed;
                    flywheel.setVelocity(targetspeed);
                } else{
                    targetspeed=0;
                    flywheel.setVelocity(0);
                }
            }
            telemetry.addLine("Flywheel Speed:"+flywheelspeed+" encoder ticks/s, "+flywheelspeed*60/28+" RPM");
            telemetry.addLine("Flywheel Speed:"+targetspeed+" encoder ticks/s, "+targetspeed*60/28+" RPM");
            telemetry.addLine("Flywheel Speed:"+flywheel.getVelocity()+" encoder ticks/s, "+flywheel.getVelocity()*60/28+" RPM");
            //spindexer
            if (spindexerpos-gamepad2.left_stick_x*spindexerDialation>=0 && spindexerpos-gamepad2.left_stick_x*spindexerDialation<=0.75){
                spindexerpos-=gamepad2.left_stick_x*spindexerDialation;
            }
            else if (spindexerpos-gamepad2.left_stick_x*spindexerDialation<0){
                spindexerpos=0;
            }
            else if (spindexerpos-gamepad2.left_stick_x*spindexerDialation>0.75){
                spindexerpos=0.75;
            }
            if (gamepad2.rightBumperWasPressed() || gamepad1.rightBumperWasPressed()){
                spindexerAutoPos++;
                spindexerpos=spindexerpositions[spindexerAutoPos%2];
            }
//            if ((spindexer.getPosition()<=spindexerpositions[0]+epsilon) && (spindexer.getPosition()>=spindexerpositions[0]-epsilon) && spindexerAutoPos%2==0){
//                spindexerAutoPos++;
//                spindexerpos=spindexerpositions[spindexerAutoPos%2];
//            }
            telemetry.addLine("outtakePos:"+spindexerAutoPos+"("+spindexerpositions[spindexerAutoPos%2]+")");
            spindexer.setPosition(spindexerpos);
            previousgamepad1.copy(gamepad1);
            //transfer
            if (gamepad2.xWasPressed()){
                transferToggle=!transferToggle;
            }
            if (transferToggle) {
                transfer.setPower(1);
                telemetry.addLine("Transfer Position: Up");
            }else{
                transfer.setPower(0);
                telemetry.addLine("Transfer Position: Stopped");
            }
            telemetry.addData("Spindexer Real Position:",spindexer.getPosition());
            //TODO:Change telemetry to add real transfer speed when encoder cable connected
//            telemetry.addData("transfer Target Speed:",transfer.getPosition());
            //update gamepad+telemetry
            previousgamepad2.copy(gamepad2);
            //hood
            hoodServo.setPower(-gamepad2.left_stick_y);

            //intake
            intake.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            //below is drivetrain
            // Mecanum drive is controlled with three axes: drive (front-and-back),
            // strafe (left-and-right), and twist (rotating the whole chassis).
            //Default:0.7
            double drive  = -gamepad1.left_stick_y*0.7;
            final double strafe_speed=0.7;
            final double drive_speed=0.7;
            //Default:0.5
            double strafe = -gamepad1.left_stick_x*0.5;
            if (gamepad1.dpad_left){
                strafe=strafe_speed;
            }
            if (gamepad1.dpad_right){
                strafe=-strafe_speed;
            }
            if (gamepad1.dpad_up){
                drive=drive_speed;
            }
            if (gamepad1.dpad_down){
                drive=-drive_speed;
            }
            double secondaryDialation=0.25;
            if (gamepad1.y) {
                drive=drive_speed* secondaryDialation;
            }
            if (gamepad1.a) {
                drive=-drive_speed*secondaryDialation;
            }
            if (gamepad1.b) {
                strafe=-strafe_speed*secondaryDialation;
            }
            if (gamepad1.x) {
                strafe=strafe_speed*secondaryDialation;
            }
            double twist  = -gamepad1.right_stick_x*0.5;
            telemetry.addData("drive: ", drive);
            telemetry.addData("strafe: ", strafe);
            telemetry.addData("twist: ", twist);

            double[] speeds = {
                    (drive - strafe - twist), //forward-left motor
                    (drive + strafe + twist), //forward-right motor
                    (drive + strafe - twist), //back-left motor
                    (drive - strafe + twist)  //back-right motor
            };

            // Loop through all values in the speeds[] array and find the greatest
            // *magnitude*.  Not the greatest velocity.
            double max = Math.abs(speeds[0]);
            for(int i = 0; i < speeds.length; i++) {
                if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
            }

            // If and only if the maximum is outside of the range we want it to be,
            // normalize all the other speeds based on the given speed value.
            if (max > 1) {
                for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
            }

            // apply the calculated values to the motors.
            leftFront.setPower(speeds[0]);
            rightFront.setPower(speeds[1]);
            leftBack.setPower(speeds[2]);
            rightBack.setPower(speeds[3]);
            telemetry.update();
        }
    }
}