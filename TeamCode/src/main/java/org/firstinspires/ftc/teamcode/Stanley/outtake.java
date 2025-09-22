package org.firstinspires.ftc.teamcode.Stanley;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class outtake {
    public boolean autoaim(){
        return false;
    }
    public void transfer(Servo linearActuator){
        int normalPos=0;
        int transferPos=1;
        linearActuator.setPosition(transferPos);
        linearActuator.setPosition(normalPos);
    }
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
