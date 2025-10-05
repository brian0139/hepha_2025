package org.firstinspires.ftc.teamcode.Stanley;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class outtake {
    //Camera position info
    Position cameraPosition=new Position(DistanceUnit.INCH,0,0,0,0);
    YawPitchRollAngles cameraOrientation=new YawPitchRollAngles(AngleUnit.DEGREES,0,-90,0,0);
    //Team color
    String teamColor;
    //April tag processor
    AprilTagProcessor apriltag;
    //Vision portal
    VisionPortal visionPortal;

    public void init(){
        //Init processor
        this.apriltag=new AprilTagProcessor.Builder().build();
        this.apriltag.setDecimation(2);
        //Init vision portal
        this.visionPortal=new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class,"Webcam1"))
                .addProcessor(apriltag)
                .build();
        //wait for camera to begin streaming
        while(this.visionPortal.getCameraState()!=VisionPortal.CameraState.STREAMING){
            try {
                sleep(20);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        //set camera exposure settings

    }
    /**
     * Autoaim to april tag.
     * Currently using built-in april tag ID process.
     * Includes drivetrain control+emergency cancel switch.
     * Target tag set by teamColor variable in class, "Red" or "Blue"
     * @param targetDistance Desired distance to tag in inches.
     * @param fl DcMotor for front-left motor
     * @param fr DcMotor for front-right motor
     * @param bl DcMotor for back-left motor
     * @param br DcMotor for back-right motor
     */
    public void autoaim(int targetDistance, DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br){
        int targetAprilTagNumber;
        //set target april tag number to aim at depending on team color.
        if (this.teamColor=="Red"){
            targetAprilTagNumber=24;
        }
        else if (this.teamColor=="Blue") {
            targetAprilTagNumber = 20;
        }
        //List for found tags
        List<AprilTagDetection> detections=apriltag.getDetections();
        //If target tag has been located
        boolean tagFound=false;
        //Main loop
        while (!gamepad1.b){
            if (!tagFound){

            }
        }
    }

    /**
     * Transfer artifact to flywheel
     * @param linearActuator Servo Linear Actuator to push artifact
     */
    public void transfer(Servo linearActuator){
        int normalPos=0;
        int transferPos=1;
        linearActuator.setPosition(transferPos);
        linearActuator.setPosition(normalPos);
    }

    /**
     * Spin flywheel to speed
     * @param targetSpeed Target flywheel speed in encoder ticks/sec.
     * @param flywheelDrive DcMotor object for flywheel motor
     * @param tolerance Tolerance in ticks/sec. from target speed to return true.
     * @return If flywheel is up to speed.
     */
    public boolean spin_flywheel(int targetSpeed, DcMotor flywheelDrive, int tolerance){
        DcMotorEx flywheelDriveEx=(DcMotorEx) flywheelDrive;
        flywheelDriveEx.setVelocity(targetSpeed);
        if (targetSpeed-tolerance<=flywheelDriveEx.getVelocity() && flywheelDriveEx.getVelocity()<=targetSpeed+tolerance){
            return true;
        }
        else{
            return false;
        }
    }
}
