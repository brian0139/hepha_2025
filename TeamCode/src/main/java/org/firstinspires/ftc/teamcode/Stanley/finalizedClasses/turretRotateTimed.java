package org.firstinspires.ftc.teamcode.Stanley.finalizedClasses;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class turretRotateTimed {
    boolean running=false;
    CRServo turretServo;
    ElapsedTime timer=new ElapsedTime();
    double power;
    double time;
    public turretRotateTimed(CRServo turretServo){
        this.turretServo=turretServo;
    }
    public boolean updatespin(double power,double time){
        if (!running){
            running=true;
            timer.reset();
        }
        turretServo.setPower(power);
        if (timer.milliseconds()>=time){
            turretServo.setPower(0);
            running=false;
            return true;
        }
        return false;
    }
}
