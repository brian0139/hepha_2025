package org.firstinspires.ftc.teamcode.Stanley.finalizedClasses;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDspindexer {
    public double Kp;
    public double Ki;
    public double Kd;
    public double P;
    public double I;
    public double D;
    public double error;
    public double lastError=0;
    public double integral;
    public double derivative;
    public double maxOutput=1;
    public double minOutput=-1;
    public double alpha=0.75;
    public double power;
    ElapsedTime timer=new ElapsedTime();

    /**
     * Constructor
     * @param Kp Proportional gain
     * @param Ki Integral gain
     * @param Kd Derivative gain
     */
    public PIDspindexer(double Kp, double Ki, double Kd){
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
        P=Kp*error;
        integral=alpha*error+(1-integral)*integral;
        I=Ki*integral;
        derivative=(error-lastError)/dt;
        D=Kd*derivative;
        double candidateOutput=P+I+D;
        //Integral conditional clamping
        if (!(candidateOutput>=maxOutput && error>0) && !(candidateOutput<=minOutput && error<0)) {
            integral += error * dt;
            I=Ki*integral;
        }
        lastError=error;
        //Reset timer for dt
        timer.reset();
        power=P+I+D;
        return P+I+D;
    }

    /**
     * Update PID output + values
     * @param error Error value
     * @return PID output
     */
    public double update(double error){
        double dt=timer.seconds();
        P=Kp*error;
        integral += error * dt;
        I=Ki*integral;
        derivative=(error-lastError)/dt;
        D=Kd*derivative;
        lastError=error;
        //Reset timer for dt
        timer.reset();
        power=P+I+D;
        return P+I+D;
    }
}
