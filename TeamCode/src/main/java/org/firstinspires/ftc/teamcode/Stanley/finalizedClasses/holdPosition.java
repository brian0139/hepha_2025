package org.firstinspires.ftc.teamcode.Stanley.finalizedClasses;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class holdPosition {
    MecanumDrive drive;
    public Pose2d initialPosition;
    public Pose2d currentPosition;
    private DcMotor leftFront=null;
    private DcMotor leftBack=null;
    private DcMotor rightFront=null;
    private DcMotor rightBack=null;

    /**
     * Constructor
     * @param drive MecanumDrive object for robot
     * @param leftFront Drivetrain motor
     * @param leftBack Drivetrain motor
     * @param rightFront Drivetrain motor
     * @param rightBack Drivetrain motor
     */
    public holdPosition(MecanumDrive drive, DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack){
        this.drive=drive;
        this.leftFront=leftFront;
        this.leftBack=leftBack;
        this.rightFront=rightFront;
        this.rightBack=rightBack;
        drive.updatePoseEstimate();
        this.initialPosition=drive.localizer.getPose();
    }

    /**
     * Set initial position to robot current position
     * @return Pose2d value initialPosition has been set to
     */
    public Pose2d setInitialPosition(){
        drive.updatePoseEstimate();
        initialPosition=drive.localizer.getPose();
        return initialPosition;
    }

    /**
     * Update stored current position of robot
     * @return Pose2d of Current position
     */
    public Pose2d updateCurrentPosition(){
        drive.updatePoseEstimate();
        currentPosition=drive.localizer.getPose();
        return currentPosition;
    }
}
