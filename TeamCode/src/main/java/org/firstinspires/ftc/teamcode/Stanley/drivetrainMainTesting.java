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

public class drivetrainMainTesting extends LinearOpMode{
    // drivetrain wheel motor declaration
    private DcMotor leftFront=null;
    private DcMotor leftBack=null;
    private DcMotor rightFront=null;
    private DcMotor rightBack=null;
    //other motors
    private DcMotorEx flywheel=null;
    private DcMotorEx intake=null;
    //servos
    private CRServo hoodServo=null;
    private Servo spindexer=null;
    private Servo transfer=null;
    //sensitivity(& other configs)
    double hoodspeed=0.5;
    int flywheelSensitivity=10;
    //vars
    int flywheelspeed=2000;
    int targetspeed=0;
    boolean flywheelToggle=false;
    //false=intake, true=outtake
    boolean spindexerPosition=false;
    double[] outtakeslots = {0.26,0.65,1};
    double[] intakeslots = {0.05,0.44,0.83};
    double[] transferpositions ={0.6,0.9};
    int outtakepos=0;
    int intakepos=0;
    boolean pasty=false;
    //button state storage
    Gamepad previousgamepad2 =new Gamepad();

    //main loop
    @Override
    public void runOpMode() {
        //initiate drivetrain motors
        leftFront   = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack    = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront   = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack   = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //other motors
        flywheel=(DcMotorEx) hardwareMap.get(DcMotor.class,"flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        intake=(DcMotorEx) hardwareMap.get(DcMotor.class,"intake");
        //servos
        hoodServo=hardwareMap.get(CRServo.class,"hoodServo");
        spindexer=hardwareMap.get(Servo.class,"spindexerServo");
        transfer=hardwareMap.get(Servo.class,"transferServo");
        //initialize spindexer
        spindexer.setPosition(intakeslots[0]);
        //telemetry message to signify robot waiting
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        //wait for driver to press play
        waitForStart();
        //repeat until opmode ends
        while (opModeIsActive()){
            //flywheel
            if (gamepad2.dpad_up){
                flywheelspeed+=flywheelSensitivity;
            }
            else if (gamepad2.dpad_down && flywheelspeed-flywheelSensitivity>=0){
                flywheelspeed-=flywheelSensitivity;
            }
            else if (gamepad2.dpad_down && flywheelspeed-flywheelSensitivity<0){
                flywheelspeed=0;
            }
            //toggle
            if (gamepad2.y && !pasty){
                telemetry.addLine("triggered");
                pasty=true;
                flywheelToggle=!flywheelToggle;
                if (flywheelToggle) {
                    targetspeed=flywheelspeed;
                    flywheel.setVelocity(targetspeed);
                } else{
                    targetspeed=0;
                    flywheel.setVelocity(0);
                }
            }
            else if (!gamepad2.y && pasty){
                telemetry.addLine("not triggered");
                pasty=false;
            }
            telemetry.addLine("Flywheel Speed:"+flywheelspeed+" encoder ticks/s, "+flywheelspeed*60/28+" RPM");
            telemetry.addLine("Flywheel Speed:"+targetspeed+" encoder ticks/s, "+targetspeed*60/28+" RPM");
            telemetry.addLine("Flywheel Speed:"+flywheel.getVelocity()+" encoder ticks/s, "+flywheel.getVelocity()*60/28+" RPM");
            //spindexer
            if (gamepad2.right_bumper && !previousgamepad2.right_bumper){
                outtakepos++;
                spindexer.setPosition(outtakeslots[outtakepos%3]);
                spindexerPosition=true;
            }
            if (gamepad2.left_bumper && !previousgamepad2.left_bumper){
                intakepos++;
                spindexer.setPosition(intakeslots[intakepos%3]);
                spindexerPosition=false;
            }
            if (spindexerPosition){
                telemetry.addData("Spindexer Position","Intake");
            }else{
                telemetry.addData("Spindexer Position","Outtake");
            }
            //update gamepad+telemetry
            previousgamepad2.copy(gamepad2);
            telemetry.addLine("outtakePos:"+outtakepos+"("+outtakeslots[outtakepos%3]+")");
            telemetry.addLine("intakePos:"+intakepos+"("+intakeslots[intakepos%3]+")");
            //hood
            if (gamepad2.dpad_right){
                hoodServo.setPower(hoodspeed);
            }
            else if (gamepad2.dpad_left){
                hoodServo.setPower(-hoodspeed);
            }
            else{
                hoodServo.setPower(0);
            }
            //transfer
            if (gamepad2.x) {
                transfer.setPosition(transferpositions[0]);
                telemetry.addLine("Transfer Position: Up ("+transferpositions[0]+")");
            }else{
                transfer.setPosition(transferpositions[1]);
                telemetry.addLine("Transfer Position: Down ("+transferpositions[1]+")");
            }

            //intake
            intake.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
            //below is drivetrain
            // Mecanum drive is controlled with three axes: drive (front-and-back),
            // strafe (left-and-right), and twist (rotating the whole chassis).
            //Default:0.7
            double drive  = -gamepad2.left_stick_y*0.7;
            //Default:0.5
            double strafe = -gamepad2.left_stick_x*0.5;
            double twist  = -gamepad2.right_stick_x*0.5;
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