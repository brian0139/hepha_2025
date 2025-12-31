package org.firstinspires.ftc.teamcode.Brian;

import android.text.method.Touch;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Alvin.colorSensor;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.PID;

public class spindexerColor {
    public CRServo spindexerServo = null;
    public DcMotor intake=null;
    ElapsedTime timer = new ElapsedTime();
    colorSensor outtakesensor;
    colorSensor intakesensor;
    AnalogInput spindexerSensor;
    public double kp = 0.00;
    public double ki = 0.00;
    public double kd = 0.00;
    PID spindexerPID = new PID(kp, ki, kd);
    double detectedLocation = 0;
    boolean lastDetected=false;
    boolean detectedLastLoop = false;
    public int[] spindexerSlots = {0, 0, 0}; //0 none, 1 green, 2 purple
    public int[] dummyMotif = {1, 2, 2};
    public int[] currentMotifPattern = null;

    public spindexerColor(CRServo spindexerServo, DcMotor intake, HardwareMap hardwareMap) {
        this.spindexerServo = spindexerServo;
        this.intake = intake;
        outtakesensor = new colorSensor(hardwareMap, "outtakeSensor");
        intakesensor = new colorSensor(hardwareMap, "intakeSensor");

    }


    public boolean spinToMotif(int motifIndex) {
        double epsilon = 0.01;
        int nextmotif = dummyMotif[motifIndex];
        if (outtakesensor.getDetected() != nextmotif) {
            spindexerServo.setPower(0.75);
        } else {
            detectedLocation = spindexerSensor.getVoltage();
            spindexerPID.init();
        }
        if (detectedLocation != -1) {
            spindexerServo.setPower(spindexerPID.update(detectedLocation - spindexerSensor.getVoltage()));
            return (spindexerSensor.getVoltage() >= detectedLocation - epsilon && spindexerSensor.getVoltage() <= detectedLocation + epsilon);
        }
        return false;
    }

    public void initSpin() {
        detectedLocation = -1;
        timer.reset();
        lastDetected=false;
    }

    public boolean spinToIntake(int motifIndex) {
        double epsilon = 0.01;
        int nextmotif = dummyMotif[motifIndex];
//        intake.setPower(0.75);
        if (intakesensor.getDetected()==0){
            if (timer.milliseconds()>=400 && lastDetected && timer.milliseconds()<=1000){
                detectedLocation=spindexerSensor.getVoltage();
                spindexerPID.init();
            }else{
                timer.reset();
                lastDetected=true;
            }
        }
        if (detectedLocation!=-1){
            spindexerServo.setPower(spindexerPID.update(detectedLocation-spindexerSensor.getVoltage()));
            return (spindexerSensor.getVoltage()>=detectedLocation-epsilon&&spindexerSensor.getVoltage()<=detectedLocation+epsilon);
        }
        return false;
    }
    public boolean spinAfterIntake(boolean intakesuccess){
        double epsilon =0.01;
        if (intakesuccess){
            spindexerServo.setPower(0.75);
        }else{
            detectedLocation=spindexerSensor.getVoltage();
            spindexerPID.init();
        }
        if (detectedLocation!=-1){
            spindexerServo.setPower(spindexerPID.update(detectedLocation-spindexerSensor.getVoltage()));
            return (spindexerSensor.getVoltage()>=detectedLocation-epsilon&&spindexerSensor.getVoltage()<=detectedLocation+epsilon);
        }
        return false;
    }
}