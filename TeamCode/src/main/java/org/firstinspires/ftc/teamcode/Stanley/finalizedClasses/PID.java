package org.firstinspires.ftc.teamcode.Stanley.finalizedClasses;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    public double Kp;
    public double Ki;
    public double Kd;
    public double error;
    public double lastError=0;
    public double integral;
    public double derivative;
    public double maxOutput=1;
    public double minOutput=0;
    ElapsedTime timer=new ElapsedTime();

    /**
     * Constructor
     * @param Kp Proportional gain
     * @param Ki Integral gain
     * @param Kd Derivative gain
     */
    public PID(double Kp, double Ki, double Kd){
        this.Kp=Kp;
        this.Ki=Ki;
        this.Kd=Kd;
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
        double P=Kp*error;
        double I=Ki*integral;
        derivative=(error-lastError)/dt;
        double D=Kd*derivative;
        double candidateOutput=P+I+D;
        //Integral conditional clamping
        if (!(candidateOutput>=maxOutput && error>0) && !(candidateOutput<=minOutput && error<0)) {
            integral += error * dt;
            I=Ki*integral;
        }
        lastError=error;
        //Reset timer for dt
        timer.reset();
        return P+I+D;
    }

    /**
     * Update PID output + values
     * @param error Error value
     * @return PID output
     */
    public double update(double error){
        double dt=timer.seconds();
        double P=Kp*error;
        double I=Ki*integral;
        derivative=(error-lastError)/dt;
        double D=Kd*derivative;
        double candidateOutput=P+I+D;
        //Integral conditional clamping
        if (!(candidateOutput>=maxOutput && error>0) && !(candidateOutput<=minOutput && error<0)) {
            integral += error * dt;
            I=Ki*integral;
        }
        lastError=error;
        //Reset timer for dt
        timer.reset();
        return P+I+D;
    }
}
