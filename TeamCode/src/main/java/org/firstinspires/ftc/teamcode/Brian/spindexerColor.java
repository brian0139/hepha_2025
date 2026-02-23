package org.firstinspires.ftc.teamcode.Brian;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Alvin.colorSensor;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.PID;

public class spindexerColor {
    //==================== Hardware ====================
    public CRServo spindexerServo = null;
    public DcMotor intake = null;
    public DcMotorEx spindexerSensor;
    public colorSensor outtakesensor;
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
    public int[] motifPattern = {1, 2, 2};
    public int[] currentMotifPattern = null;
    public int[] touchSensorEncoder = {1832,4616,7281};
    public int encoderOffset=0;

    //==================== Configs ====================
    public double adjust = 0.1;
    public double spinspeed = 0.5;
    public double mindetectiontime = 300;
    public double maxdetectiontime = 500;
    static final double CSIntegrationTimeMS = 100;
    public double autoSpinEpsilon=150;
    public double[] kS = {0.0007, 0.0005, 0.00003};
    public double[] inslotsV = {0, -2691, -5470};
    public double[] outslotsV = {-6796, -1423, -4087};

    //==================== PID ====================
    public PID spindexerPID = new PID(kS[0], kS[1], kS[2]);

    public spindexerColor(CRServo spindexerServo, DcMotor intake, HardwareMap hardwareMap) {
        this.spindexerServo = spindexerServo;
        this.intake = intake;
        spindexerSensor=hardwareMap.get(DcMotorEx.class,"intake");
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

    /**
     * Calculates absolute distance of spindexer rotation
     * @param setpoint Target value
     * @param currentpoint Current value
     * @return Absolute distance (double)
     */
    public double calculateError(double setpoint, double currentpoint){
        double range=8192;
        double error=(setpoint%8192)-currentpoint;

        while (error>range/2) error-=range;
        while (error<-range/2) error+=range;

        return error;
    }

    public void initSpin() {
        spindexerPID.init();
        detectedLocation = -1;
        detectioncnt=0;
        timer.reset();
        lastDetected=false;
    }

    public boolean spinToMotif(int motifIndex) {
        int nextmotif = motifPattern[motifIndex];
        if (outtakesensor.getDetected() != nextmotif && !lastDetected) {
            spindexerServo.setPower(0.75);
            detectedLocation = -1;
            lastDetected=false;
        } else if (detectedLocation==-1){
            /*TODO:Adjust for color sensor offset
            If adjust is negative,
               detectedLocation = Math.abs(spindexerSensor.getCurrentPosition()+encoderOffset-adjust);
            If adjust is positive,
               detectedLocation = (spindexerSensor.getCurrentPosition()+encoderOffset+adjust)%3.3;
            */
            detectedLocation = Math.abs(spindexerSensor.getCurrentPosition()+encoderOffset-adjust);
            lastDetected=true;
            spindexerPID.init();
        }
        if (detectedLocation != -1) {
            spindexerServo.setPower(spindexerPID.update(calculateError(detectedLocation,spindexerSensor.getCurrentPosition()+encoderOffset)));
            return (spindexerSensor.getCurrentPosition()+encoderOffset >= detectedLocation - autoSpinEpsilon && spindexerSensor.getCurrentPosition()+encoderOffset <= detectedLocation + autoSpinEpsilon);
        }
        return false;
    }

    public boolean spinToMotifV2(int motifIndex) {
        if (detectioncnt==-1) detectioncnt=0;
        if (detectioncnt==3) return false;
        int nextmotif = motifPattern[motifIndex];
        if (Math.abs(calculateError(outslotsV[currentSlot],spindexerSensor.getCurrentPosition()+encoderOffset))>autoSpinEpsilon && !waitingCSIntegration){
            spindexerServo.setPower(spindexerPID.update(calculateError(outslotsV[currentSlot],spindexerSensor.getCurrentPosition()+encoderOffset)));
        }else{
            if (!waitingCSIntegration){
                waitingCSIntegration=true;
                intakeTimer.reset();
            }
            holdSpindexerOuttake();
            if (intakeTimer.milliseconds()>=CSIntegrationTimeMS) {
                waitingCSIntegration=false;
                if (intakesensor.getDetected() == nextmotif) {
                    spindexerServo.setPower(0);
                    return true;
                }
                detectioncnt++;
                currentSlot++;
                currentSlot %= 3;
            }
        }
        return false;
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
        if (Math.abs(calculateError(inslotsV[currentSlot],spindexerSensor.getCurrentPosition()+encoderOffset))>autoSpinEpsilon && !waitingCSIntegration){
            spindexerServo.setPower(Math.min(spindexerPID.update(calculateError(inslotsV[currentSlot],spindexerSensor.getCurrentPosition()+encoderOffset)),0.75));
        }else{
            if (!waitingCSIntegration){
                waitingCSIntegration=true;
                intakeTimer.reset();
            }
            holdSpindexer();
            if (intakeTimer.milliseconds()>=CSIntegrationTimeMS) {
                waitingCSIntegration=false;
                if (intakesensor.getDetected() == 0) {
                    spindexerServo.setPower(0);
                    ballIn = true;
                    return true;
                }
                detectioncnt++;
                currentSlot++;
                currentSlot %= 3;
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
                if (spindexerSlots[i]==spindexerSensor.getCurrentPosition()+encoderOffset){
                    spindexerSlots[i]=1;
                }
            }
            return numBalls;
        }
        return 0;
    }

    public void holdSpindexer(){
        spindexerServo.setPower(spindexerPID.update(calculateError(inslotsV[currentSlot],spindexerSensor.getCurrentPosition()+encoderOffset)));
    }
    public void holdSpindexerOuttake(){
        spindexerServo.setPower(spindexerPID.update(calculateError(outslotsV[currentSlot],spindexerSensor.getCurrentPosition()+encoderOffset)));
    }

    /**
     * Rsets encoder offset
     * @param direction Direction servo is spinning()
     */
    public void resetSpindexerTouch(){
//        int tmp= spindexerSensor.getCurrentPosition()+encoderOffset;
//        tmp%=8192;
//        int absoluteOffset=99999;
//        int curSlot=-1;
//        for (int i=0;i<3;i++){
//            if (Math.abs(tmp-touchSensorEncoder[i])<absoluteOffset){
//                curSlot=i;
//                absoluteOffset=Math.abs(tmp-touchSensorEncoder[i]);
//            }
//        }
//        encoderOffset=touchSensorEncoder[curSlot];
//        spindexerSensor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        spindexerSensor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean spinAfterIntake(boolean intakesuccess){
        double epsilon =0.01;
        if (intakesuccess){
            spindexerServo.setPower(0.75);
        }else{
            detectedLocation=spindexerSensor.getCurrentPosition()+encoderOffset;
            spindexerPID.init();
        }
        if (detectedLocation!=-1){
            spindexerServo.setPower(spindexerPID.update(detectedLocation-spindexerSensor.getCurrentPosition()+encoderOffset));
            return (spindexerSensor.getCurrentPosition()+encoderOffset>=detectedLocation-epsilon&&spindexerSensor.getCurrentPosition()+encoderOffset<=detectedLocation+epsilon);
        }
        return false;
    }
}