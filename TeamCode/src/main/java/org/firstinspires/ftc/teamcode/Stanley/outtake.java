package org.firstinspires.ftc.teamcode.Stanley;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Aaron.aprilTagV3;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.Objects;

public class outtake {
    //Team color
    String teamColor;
    //April tag processor
    aprilTagV3 apriltag;
    //Outtake flywheel
    public DcMotorEx flywheelDrive;
    //Outtake Hood Servo
    CRServo hoodServo;
    //TODO: get servo RPM
    //Servo RPM
    double servoRPM=50;
    //Degrees changed for every servo rotation
    double servoDegPerRot =20;
    //transfer positions(up, down)
    public static double[] transferpositions ={0.68,0.875};
    //hood angle transitions
    //save ms time for hood
    long savemstime=0;
    //save hood angle for starting hood angle
    double savehoodAngle=0;
    //if hood running
    public boolean runninghood=false;
    //hood angle(in degrees)
    public double hoodAngle=0;
    //transfer servo
    public Servo transfer;
    //drivetrain motors
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    //auto aim vars
    //  Drive = Error * Gain
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    double TURN_GAIN   =  0.025  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    //vars
    int targetTagID=-1;
    public outtake(HardwareMap hardwareMap, DcMotorEx flywheelDrive, String teamColor, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, CRServo hoodServo, Servo transfer, boolean useTag){
        this.flywheelDrive=flywheelDrive;
        this.teamColor=teamColor;
        this.leftFront=leftFront;
        this.rightFront=rightFront;
        this.leftBack=leftBack;
        this.rightBack=rightBack;
        this.hoodServo=hoodServo;
        this.transfer=transfer;
        //set target april tag number to aim at depending on team color.
        if (Objects.equals(this.teamColor, "Red") && this.targetTagID!=-1){
            this.targetTagID=24;
        }
        else if (Objects.equals(this.teamColor, "Blue") && this.targetTagID!=-1) {
            this.targetTagID = 20;
        }
        //Init apriltag instance
        if (useTag) {
            this.apriltag = new aprilTagV3(hardwareMap);
            this.apriltag.setPipeline(4);
            this.apriltag.init();
        }
    }
    /**
     * Autoaim to april tag.
     * Currently using built-in april tag ID process.
     * Includes drivetrain control+emergency cancel switch.
     * Target tag set by teamColor variable in class, "Red" or "Blue"
     * @return False if canceled or teamColor not found, True if successful
     */
    public boolean autoturn(){
        this.apriltag.scanOnce();
        if (!apriltag.hasValidTarget()){
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            return false;
        }
        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double  headingError    = -apriltag.getYaw();

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        double drive=0;
        double twist   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
        double strafe=0;

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
        return true;
    }

    public boolean setHood(double degrees, boolean override){
        double rotations=(degrees-this.hoodAngle)/this.servoDegPerRot;
        //time needed to rotate for in ms
        double time=Math.abs(rotations*60*1000/this.servoRPM);
        //If overriding to new position and servo is already running
        if (override && runninghood){
            //update current hood angle
            this.updateHoodAngle(degrees);
            //Initialize variables for new position
            this.savemstime=System.currentTimeMillis();
            this.savehoodAngle=this.hoodAngle;
            if (rotations > 0) {
                this.hoodServo.setPower(1);
            } else if (rotations < 0) {
                this.hoodServo.setPower(-1);
            }
        }
        //Exit condition
        if (System.currentTimeMillis()-this.savemstime>=time){
            //Update hood position
            this.updateHoodAngle(degrees);
            //set runninghood to false(no longer running hood)
            this.runninghood=false;
            //stop hood servo
            this.hoodServo.setPower(0);
            return true;
        }
        if (!runninghood) {
            this.savemstime=System.currentTimeMillis();
            this.savehoodAngle=this.hoodAngle;
            if (rotations > 0) {
                this.hoodServo.setPower(1);
            } else if (rotations < 0) {
                this.hoodServo.setPower(-1);
            }
            this.runninghood=true;
        }
        return false;
    }

    /**
     * Helper function to update current angle of hood
     * @param degrees Target degrees the hood is currently trying to get to
     */
    public void updateHoodAngle(double degrees){
        double rotations=(degrees-this.hoodAngle)/this.servoDegPerRot;
        //Update hood position
        double elapsed = System.currentTimeMillis() - this.savemstime;
        double totalRotation = (elapsed * this.servoRPM) / 60000.0;
        this.hoodAngle = this.savehoodAngle + (rotations > 0 ? 1 : -1) * totalRotation * this.servoDegPerRot;
    }

    /**
     * Transfer artifact to flywheel(move transfer up)
     */
    public void transferUp(){
        this.transfer.setPosition(this.transferpositions[0]);
    }
    /**
     * Lower Transfer
     */
    public void transferDown(){
        this.transfer.setPosition(this.transferpositions[1]);
    }

    /**
     * Spin flywheel to speed
     * @param targetSpeed Target flywheel speed in encoder ticks/sec.
     * @param tolerance Tolerance in ticks/sec. from target speed to return true.
     * @return If flywheel is up to speed.
     */
    public boolean spin_flywheel(double targetSpeed, int tolerance){
        DcMotorEx flywheelDriveEx=this.flywheelDrive;
        flywheelDriveEx.setVelocity(targetSpeed);
        if (targetSpeed-tolerance<=flywheelDriveEx.getVelocity() && flywheelDriveEx.getVelocity()<=targetSpeed+tolerance){
            return true;
        }
        else{
            return false;
        }
    }
}
