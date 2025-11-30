package org.firstinspires.ftc.teamcode.Stanley;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Math;
@TeleOp

public class drivetrainMain extends LinearOpMode{
    // drivetrain wheel motor declaration
    private DcMotor leftFront=null;
    private DcMotor leftBack=null;
    private DcMotor rightFront=null;
    private DcMotor rightBack=null;
    //other motors
    private DcMotorEx flywheel=null;
    private DcMotor intake=null;
    //servos
    private CRServo hoodServo=null;
    private Servo spindexer=null;
    private Servo transfer=null;
    //sensitivity
    double flywheelSensitivity=10;
    //vars
    int flywheelspeed=0;
    int targetspeed=0;
    boolean flywheelToggle=false;
    double[] outtakeslots = {0.26,0.65,1};
    double[] intakeslots = {0.05,0.44,0.83};
    double[] transferpositions ={0.6,0.9};
    int outtakepos=0;
    int intakepos=0;
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
        //other motors
        flywheel=hardwareMap.get(DcMotorEx.class,"flywheel");
        intake=hardwareMap.get(DcMotor.class,"intake");
        //servos
        hoodServo=hardwareMap.get(CRServo.class,"hoodServo");
        spindexer=hardwareMap.get(Servo.class,"spindexerServo");
        transfer=hardwareMap.get(Servo.class,"transferServo");

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
            if (gamepad2.y && !previousgamepad2.y && !flywheelToggle){
                targetspeed=flywheelspeed;
                flywheelToggle=true;
                flywheel.setVelocity(targetspeed);
            }else if (gamepad2.y && !previousgamepad2.y && flywheelToggle){
                flywheelToggle=false;
                targetspeed=0;
                flywheel.setVelocity(0);
            }
            telemetry.addData("Flywheel Speed(encoder ticks/s):",flywheelspeed);
            telemetry.addData("Flywheel Target Velocity(encoder ticks/s):",targetspeed);
            telemetry.addData("Flywheel Real Velocity(encoder ticks/s):",flywheel.getVelocity());
            //spindexer
            if (gamepad2.right_bumper && !previousgamepad2.right_bumper){
                outtakepos++;
                spindexer.setPosition(outtakeslots[outtakepos%3]);
            }
            telemetry.addLine("outtakePos:"+outtakepos+"("+outtakeslots[outtakepos%3]+")");
            if (gamepad2.left_bumper && !previousgamepad2.left_bumper){
                intakepos++;
                spindexer.setPosition(intakeslots[intakepos%3]);
            }
            telemetry.addLine("intakePos:"+intakepos+"("+intakeslots[intakepos%3]+")");
            //hood
            hoodServo.setPower(gamepad2.left_stick_y);
            //transfer
            if (gamepad2.b) {
                transfer.setPosition(transferpositions[1]);
                telemetry.addLine("Transfer Position: Up ("+transferpositions[1]+")");
            }else{
                transfer.setPosition(transferpositions[0]);
                telemetry.addLine("Transfer Position: Down ("+transferpositions[0]+")");
            }

            //intake
            intake.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            //below is drivetrain
            // Mecanum drive is controlled with three axes: drive (front-and-back),
            // strafe (left-and-right), and twist (rotating the whole chassis).
            //Default:0.7
            double drive  = gamepad1.right_stick_y*0.7;
            final double strafe_speed=0.5;
            //Default:0.5
            double strafe = -gamepad1.right_stick_x*0.5;
            if (gamepad1.dpad_left){
                strafe=strafe_speed;
            }
            else if (gamepad1.dpad_right){
                strafe=-strafe_speed;
            }
            double twist  = -gamepad1.left_stick_x*0.5;
            telemetry.addData("drive: ", drive);
            telemetry.addData("strafe: ", strafe);
            telemetry.addData("twist: ", twist);

            double[] speeds = {
                    (drive - strafe + twist), //forward-left motor
                    (drive + strafe + twist), //forward-right motor
                    (-drive - strafe + twist), //back-left motor
                    (-drive + strafe + twist)  //back-right motor
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
            rightBack.setPower(speeds[1]);
            leftBack.setPower(speeds[2]);
            rightFront.setPower(speeds[3]);
            //update gamepad+telemetry
            previousgamepad2.copy(gamepad2);
            telemetry.update();
        }
    }
}