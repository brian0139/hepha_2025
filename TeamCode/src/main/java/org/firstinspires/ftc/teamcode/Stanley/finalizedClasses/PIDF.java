package org.firstinspires.ftc.teamcode.Stanley.finalizedClasses;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDF {
    public double Kp;
    public double Ki;
    public double Kd;
    public double Kf;
    public double P;
    public double I;
    public double D;
    public double F;
    public double error;
    public double lastError=0;
    public double integral;
    public double derivative;
    public double maxOutput=1;
    public double minOutput=-1;
    public double power;
    ElapsedTime timer=new ElapsedTime();

    /**
     * Constructor
     * @param Kp Proportional gain
     * @param Ki Integral gain
     * @param Kd Derivative gain
     */
    public PIDF(double Kp, double Ki, double Kd, double Kf){
        this.Kp=Kp;
        this.Ki=Ki;
        this.Kd=Kd;
        this.Kf=Kf;
    }

    /**
     * Initialize PID timer
     */
    public void init(){
        timer.reset();
    }

    /**
     * Update PID output + values
     * @param target Target value
     * @param current Current value
     * @return PID output
     */
    public double update(double target, double current){
        double dt=timer.seconds();
        error=target-current;
        P=Kp*error;
        I=Ki*integral;
        derivative=(error-lastError)/dt;
        D=Kd*derivative;
        F=Kf*target;
        double candidateOutput=P+I+D+F;
        //Integral conditional clamping
        if (!(candidateOutput>=maxOutput && error>0) && !(candidateOutput<=minOutput && error<0)) {
            integral += error * dt;
            I=Ki*integral;
        }
        lastError=error;
        //Reset timer for dt
        timer.reset();
        power=P+I+D+F;
        return P+I+D+F;
    }
}
