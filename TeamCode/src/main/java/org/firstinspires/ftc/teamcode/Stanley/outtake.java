package org.firstinspires.ftc.teamcode.Stanley;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Aaron.aprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.Objects;

public class outtake {
    //Team color
    String teamColor;
    //April tag processor
    aprilTag apriltag;
    //Outtake flywheel
    DcMotorEx flywheelDrive;
    //Outtake Hood Servo
    CRServo hoodServo;
    //Servo RPM
    double servoRPM=50;
    //Degrees changed for every servo rotation
    double servoDegPerRot =10;
    //transfer positions(up, down)
    double[] transferpositions ={0.62,0.875};
    //hood angle transitions
    //save ms time for hood
    long savemstime=0;
    //if hood running
    boolean runninghood=false;
    //hood angle(in degrees)
    double hoodAngle=0;
    //transfer servo
    Servo transfer;
    //drivetrain motors
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    //auto aim vars
    //  Drive = Error * Gain
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    //vars
    int targetTagID=-1;
    public outtake(HardwareMap hardwareMap, DcMotorEx flywheelDrive, String teamColor, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, CRServo hoodServo, Servo transfer){
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
        this.apriltag=new aprilTag(hardwareMap);
        this.apriltag.init();
    }
    /**
     * Autoaim to april tag.
     * Currently using built-in april tag ID process.
     * Includes drivetrain control+emergency cancel switch.
     * Target tag set by teamColor variable in class, "Red" or "Blue"
     * @param targetDistance Desired distance to tag in inches.
     * @return False if canceled or teamColor not found, True if successful
     */
    public boolean autoaim(int targetDistance){
        this.apriltag.scanOnce();
        List<AprilTagDetection> detections=this.apriltag.getAllDetections();
        AprilTagDetection targetDetection=null;
        boolean foundtag=false;
        for (AprilTagDetection detection : detections){
            if (detection.id==this.targetTagID){
                targetDetection=detection;
                foundtag=true;
            }
        }
        if (!foundtag){
            return false;
        }
        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double  rangeError      = (targetDetection.ftcPose.range - targetDistance);
        double  headingError    = targetDetection.ftcPose.bearing;
        double  yawError        = targetDetection.ftcPose.yaw;

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        double drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
        double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        double[] speeds = {
                (drive - strafe + turn), //forward-left motor
                (drive + strafe + turn), //forward-right motor
                (-drive - strafe + turn), //back-left motor
                (-drive + strafe + turn)  //back-right motor
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
        return true;
    }

    public boolean setHood(double degrees){
        double rotations=degrees-this.hoodAngle/this.servoDegPerRot;
        //time needed to rotate for in ms
        double time=Math.abs(rotations*60*1000/this.servoRPM);
        if (runninghood){
            if (rotations>0) this.hoodAngle += ((double) (System.currentTimeMillis() - savemstime)*this.servoRPM*this.servoDegPerRot) / (60*1000);
            else if (rotations<0) this.hoodAngle -= ((double) (System.currentTimeMillis() - savemstime)*this.servoRPM*this.servoDegPerRot) / (60*1000);
        }
        if (System.currentTimeMillis()-savemstime>=time){
            this.runninghood=false;
            return true;
        }
        if (!runninghood) {
            this.savemstime=System.currentTimeMillis();
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
