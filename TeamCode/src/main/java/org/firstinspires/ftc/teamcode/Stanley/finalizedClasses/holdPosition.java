package org.firstinspires.ftc.teamcode.Stanley.finalizedClasses;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class holdPosition {
    MecanumDrive drive;
    public Pose2d initialPosition;
    public Pose2d currentPosition;
    private DcMotor leftFront=null;
    private DcMotor leftBack=null;
    private DcMotor rightFront=null;
    private DcMotor rightBack=null;
    //PID variables {Kp,Ki,Kd}
    double[] Kx={0,0,0};
    double[] Ky={0,0,0};
    double[] Kt={0,0,0};
    PID xPID=new PID(Kx[0],Kx[1],Kx[2]);
    PID yPID=new PID(Ky[0],Ky[1],Ky[2]);
    PID tPID=new PID(Kt[0],Kt[1],Kt[2]);
    double powerX;
    double powerY;
    double powerT;

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

    public double hold(){
        updateCurrentPosition();
        powerX=xPID.update(initialPosition.position.x,currentPosition.position.x);
        powerY=yPID.update(initialPosition.position.y,currentPosition.position.y);
        double errorT= initialPosition.heading.imag * currentPosition.heading.real - initialPosition.heading.real * currentPosition.heading.imag;
        powerT=tPID.update(errorT);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(powerX,powerY),powerT));
        return Math.sqrt((initialPosition.position.x-currentPosition.position.x)*(initialPosition.position.x-currentPosition.position.x)+(initialPosition.position.y-currentPosition.position.y)*(initialPosition.position.y-currentPosition.position.y));
    }

    /**
     * Set initial position to robot current position
     * @return Pose2d value initialPosition has been set to
     */
    public Pose2d setInitialPosition(){
        drive.updatePoseEstimate();
        initialPosition=drive.localizer.getPose();
        xPID.init();
        yPID.init();
        tPID.init();
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
