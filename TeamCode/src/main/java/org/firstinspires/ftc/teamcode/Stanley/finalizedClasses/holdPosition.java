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
    //PID variables {Kp,Ki,Kd}
    public double[] Kx={0,0,0};
    public double[] Ky={0,0,0};
    public double[] Kt={0,0,0};
    PID xPID=new PID(Kx[0],Kx[1],Kx[2]);
    PID yPID=new PID(Ky[0],Ky[1],Ky[2]);
    PID tPID=new PID(Kt[0],Kt[1],Kt[2]);
    double powerX;
    double powerY;
    double powerT;

    /**
     * Constructor
     * @param drive MecanumDrive object for robot
     */
    public holdPosition(MecanumDrive drive){
        this.drive=drive;
        drive.updatePoseEstimate();
        this.initialPosition=this.drive.localizer.getPose();
        this.currentPosition=this.drive.localizer.getPose();
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
