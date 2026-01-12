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
    public double adjust=0.1;
    ElapsedTime timer = new ElapsedTime();
    colorSensor outtakesensor;
    public colorSensor intakesensor;
    AnalogInput spindexerSensor;
    public double[] kS={1.1,1.5,0.017};
    public double[] inslotsV={0.678,1.755,2.833};
    public double[] outslotsV={2.285,0.131,1.2};
    int numGreen=0;
    int numPurple=0;

    public PID spindexerPID = new PID(kS[0], kS[1], kS[2]);
    double detectedLocation = 0;
    public double mindetectiontime=300;
    public double maxdetectiontime=500;
    public double spinspeed=0.35;
    boolean lastDetected=false;
    public int detectioncnt=-1;
    boolean detectedLastLoop = false;
    public int[] spindexerSlots = {0, 0, 0}; //0 none, 1 green, 2 purple
    public int currentSlot=0;
    public int[] dummyMotif = {1, 2, 2};
    public int[] currentMotifPattern = null;

    public spindexerColor(CRServo spindexerServo, DcMotor intake, HardwareMap hardwareMap) {
        this.spindexerServo = spindexerServo;
        this.intake = intake;
        spindexerSensor=hardwareMap.get(AnalogInput.class,"spindexerAnalog");
        outtakesensor = new colorSensor(hardwareMap, "outtakeSensor");
        intakesensor = new colorSensor(hardwareMap, "intakeSensor");
    }

    public void getColor(){
        if (intakesensor.isGreen()){
            numGreen++;
        }else if (intakesensor.isPurple()){
            numPurple++;
        }
    }


    public boolean spinToMotif(int motifIndex) {
        double epsilon = 0.01;
        int nextmotif = dummyMotif[motifIndex];
        if (outtakesensor.getDetected() != nextmotif && !lastDetected) {
            spindexerServo.setPower(0.75);
            detectedLocation = -1;
            lastDetected=false;
        } else if (detectedLocation==-1){
            /*TODO:Adjust for color sensor offset
            If adjust is negative,
               detectedLocation = Math.abs(spindexerSensor.getVoltage()-adjust);
            If adjust is positive,
               detectedLocation = (spindexerSensor.getVoltage()+adjust)%3.3;
            */
            detectedLocation = Math.abs(spindexerSensor.getVoltage()-adjust);
            lastDetected=true;
            spindexerPID.init();
        }
        if (detectedLocation != -1) {
            spindexerServo.setPower(spindexerPID.update(calculateError(detectedLocation,spindexerSensor.getVoltage())));
            return (spindexerSensor.getVoltage() >= detectedLocation - epsilon && spindexerSensor.getVoltage() <= detectedLocation + epsilon);
        }
        return false;
    }
    public double calculateError(double setpoint, double currentpoint){
        double range=3.3;
        double error=setpoint-currentpoint;

        while (error>range/2) error-=range;
        while (error<-range/2) error+=range;

        return error;
    }

    public void initSpin() {
        detectedLocation = -1;
        timer.reset();
        lastDetected=false;
        spindexerServo.setPower(spinspeed);
    }

    /**
     * Spin empty slot on spindexer to intake
     * @return False if searching, true if sucessful/searched all slots
     */
    public boolean spinToIntake() {
        if (detectioncnt==-1){
            detectioncnt=0;
        }
        if (detectioncnt==3){
            return true;
        }
        double epsilon = 0.01;
        if (!((spindexerSensor.getVoltage()>=inslotsV[currentSlot]-epsilon)&&(spindexerSensor.getVoltage()<=inslotsV[currentSlot]+epsilon))){
            spindexerServo.setPower(Math.max(spindexerPID.update(calculateError(inslotsV[currentSlot],spindexerSensor.getVoltage())),0.75));
        }else{
            detectioncnt++;
            currentSlot++;
            currentSlot%=3;
            if (intakesensor.getDetected()==0){
                spindexerServo.setPower(0);
                return true;
            }
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