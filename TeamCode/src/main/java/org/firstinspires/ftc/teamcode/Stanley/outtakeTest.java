//package org.firstinspires.ftc.teamcode.Stanley;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//import java.lang.Math;
//@TeleOp
//
//public class outtakeTest extends LinearOpMode{
//    // drivetrain wheel motor declaration
//    private DcMotor leftFront=null;
//    private DcMotor leftBack=null;
//    private DcMotor rightFront=null;
//    private DcMotor rightBack=null;
//    private DcMotorEx flywheel=null;
//
//    //main loop
//    @Override
//    public void runOpMode() {
//        //initiate drivetrain motors
//        leftFront   = hardwareMap.get(DcMotor.class, "leftFront");
//        leftBack    = hardwareMap.get(DcMotor.class, "leftBack");
//        rightFront  = hardwareMap.get(DcMotor.class, "rightFront");
//        rightBack   = hardwareMap.get(DcMotor.class, "rightBack");
//        //outtake instance
//        flywheel=hardwareMap.get(DcMotorEx.class,"flywheel");
//        outtake outTake = new outtake(hardwareMap, flywheel, "Red");
//
//        //telemetry message to signify robot waiting
//        telemetry.addLine("Robot Ready.");
//        telemetry.update();
//        //wait for driver to press play
//        waitForStart();
//        //repeat until opmode ends
//        //false=encoder
//        boolean rpmorencoder=false;
//        double adjustFactorTrigger=10;
//        double adjustFactor=1;
//        double encoderticksperrev=2000;
//        double ets=0;
//        double rpm=0;
//        boolean pastRbumper=false;
//        boolean pastLbumper=false;
//        double maxets=10000;
//        double maxrpm=10000;
//        while (opModeIsActive()){
//            //outtake test
//            if (gamepad1.right_bumper && !pastRbumper){
//                pastRbumper=true;
//                if (rpmorencoder){
//                    rpm+=adjustFactor;
//                }
//                else{
//                    ets+=adjustFactor;
//                }
//            }
//            else if (!gamepad1.right_bumper && pastRbumper){
//                pastRbumper=false;
//            }
//            if (gamepad1.y){
//                rpmorencoder=true;
//            }
//            else if (gamepad1.a){
//                rpmorencoder=false;
//            }
//            if (gamepad1.left_bumper && !pastLbumper){
//                pastLbumper=true;
//                if (rpmorencoder){
//                    rpm-=adjustFactor;
//                }
//                else{
//                    ets-=adjustFactor;
//                }
//            }
//            else if (!gamepad1.left_bumper && pastLbumper){
//                pastLbumper=false;
//            }
//            if (rpmorencoder){
//                rpm+=gamepad1.right_trigger*adjustFactorTrigger;
//                rpm-=gamepad1.left_trigger*adjustFactorTrigger;
//            }
//            else{
//                ets+=gamepad1.right_trigger*adjustFactorTrigger;
//                ets-=gamepad1.left_trigger*adjustFactorTrigger;
//            }
//            //make sure values are within range+convert rpm to ets & vice versa
//            if (rpmorencoder){
//                rpm=Math.min(maxrpm,rpm);
//                rpm=Math.max(0,rpm);
//                ets=rpm*encoderticksperrev/60;
//                telemetry.addData("Target Speed:","("+ets+"encoder ticks/sec) "+rpm+"rotations/min");
//            }
//            else{
//                ets=Math.min(maxets,ets);
//                ets=Math.max(0,ets);
//                rpm=ets*60/encoderticksperrev;
//                telemetry.addData("Target Speed:",ets+"encoder ticks/sec "+"("+rpm+"rotations/min)");
//            }
//            //apply speed
//            outTake.spin_flywheel(ets,1);
//            telemetry.addData("Current Speed:", flywheel.getVelocity()+"encoder ticks/sec （"+ets*60/encoderticksperrev+"rpm)");
//            //below is drivetrain
//            // Mecanum drive is controlled with three axes: drive (front-and-back),
//            // strafe (left-and-right), and twist (rotating the whole chassis).
//            //Default:0.7
//            double drive  = gamepad1.right_stick_y*0.7;
//            final double strafe_speed=0.5;
//            //Default:0.5
//            double strafe = -gamepad1.right_stick_x*0.5;
//            if (gamepad1.dpad_left){
//                strafe=strafe_speed;
//            }
//            else if (gamepad1.dpad_right){
//                strafe=-strafe_speed;
//            }
//            double twist  = -gamepad1.left_stick_x*0.5;
//            telemetry.addData("drive: ", drive);
//            telemetry.addData("strafe: ", strafe);
//            telemetry.addData("twist: ", twist);
//            telemetry.update();
//
//            double[] speeds = {
//                    (drive - strafe + twist), //forward-left motor
//                    (drive + strafe + twist), //forward-right motor
//                    (-drive - strafe + twist), //back-left motor
//                    (-drive + strafe + twist)  //back-right motor
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
//            rightBack.setPower(speeds[1]);
//            leftBack.setPower(speeds[2]);
//            rightFront.setPower(speeds[3]);
//        }
//    }
//}