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
    //==================== Hardware ====================
    public CRServo spindexerServo = null;
    public DcMotor intake = null;
    AnalogInput spindexerSensor;
    colorSensor outtakesensor;
    public colorSensor intakesensor;

    //==================== Timers ====================
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime intakeTimer = new ElapsedTime();

    //==================== State Vars ====================
    boolean waitingCSIntegration = false;
    boolean lastDetected = false;
    boolean detectedLastLoop = false;
    public boolean ballIn = false;

    //==================== Detection Data ====================
    double detectedLocation = 0;
    public int detectioncnt = -1;
    int numGreen = 0;
    int numPurple = 0;

    //==================== Spindexer Slot State ====================
    public int[] spindexerSlots = {0, 0, 0}; //0 none, 1 green, 2 purple
    public int currentSlot = 0;
    public int[] dummyMotif = {1, 2, 2};
    public int[] currentMotifPattern = null;

    //==================== Configs ====================
    public double adjust = 0.1;
    public double spinspeed = 0.5;
    public double mindetectiontime = 300;
    public double maxdetectiontime = 500;
    static final double CSIntegrationTimeMS = 100;
    public double[] kS = {1.0, 0.013, 0.014};
    public double[] inslotsV = {2.285, 0.131, 1.2};
    public double[] outslotsV = {0.678, 1.755, 2.833};

    //==================== PID ====================
    public PID spindexerPID = new PID(kS[0], kS[1], kS[2]);

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
        detectioncnt=0;
        timer.reset();
        lastDetected=false;
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
            spindexerServo.setPower(Math.min(spindexerPID.update(calculateError(inslotsV[currentSlot],spindexerSensor.getVoltage())),0.75));
        }else{
            if (!waitingCSIntegration){
                waitingCSIntegration=true;
                intakeTimer.reset();
            }
            if (intakeTimer.milliseconds()>=CSIntegrationTimeMS) {
                waitingCSIntegration=false;
                detectioncnt++;
                currentSlot++;
                currentSlot %= 3;
                if (intakesensor.getDetected() == 0) {
                    spindexerServo.setPower(0);
                    ballIn = true;
                    return true;
                }
            }
        }
        ballIn=false;
        return false;
    }

    public int trackballs(){
        if (ballIn==true){
            int numBalls=0;
            for (int i=0;i<3;i++){
                if (spindexerSlots[i]!=0){
                    numBalls++;
                }
                if (spindexerSlots[i]==spindexerSensor.getVoltage()){
                    spindexerSlots[i]=1;
                }
            }
            return numBalls;
        }
        return 0;
    }

    public void holdSpindexer(){
        spindexerServo.setPower(Math.min(spindexerPID.update(calculateError(inslotsV[currentSlot],spindexerSensor.getVoltage())),0.75));
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