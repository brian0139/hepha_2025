package org.firstinspires.ftc.teamcode.Brian;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Alvin.colorSensor;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.PID;

@Config
public class spindexerColor {
    //==================== Hardware ====================
    public CRServo spindexerServo = null;
    public DcMotor intake = null;
    public AnalogInput spindexerAnalog = null;
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
    public double autoSpinEpsilon=150;
    public double[] kS = {0.0007, 0.0005, 0.00003};
//    public double[] kS = {1.06, 0.1, 0.014};
    public double[] inslotsV = {0, -2691, -5470};
    public double[] outslotsV = {-6796, -1423, -4087};

    //==================== Absolute Encoder Config ====================
    // Most absolute encoders used in FTC output 0..3.3V over one rotation.
    public static double ENCODER_MAX_VOLTAGE = 3.3;
    public static double ENCODER_TICKS_PER_REV = 8192.0;
    // Rotate this to make "slot 0" align with your chosen reference.
    public static double ENCODER_ZERO_TICKS = 0.0;
    public static boolean ENCODER_INVERTED = false;

    //==================== PID ====================
    public PID spindexerPID = new PID(kS[0], kS[1], kS[2]);

    public spindexerColor(CRServo spindexerServo, DcMotor intake, HardwareMap hardwareMap) {
        this.spindexerServo = spindexerServo;
        this.intake = intake;
        spindexerAnalog = hardwareMap.get(AnalogInput.class, "spindexerAnalog");
        outtakesensor = new colorSensor(hardwareMap, "outtakeSensor");
        intakesensor = new colorSensor(hardwareMap, "intakeSensor");
    }

    public double getSpindexerTicks() {
        if (spindexerAnalog == null) return 0.0;
        if (ENCODER_MAX_VOLTAGE <= 0 || ENCODER_TICKS_PER_REV <= 0) return 0.0;

        double normalized = spindexerAnalog.getVoltage() / ENCODER_MAX_VOLTAGE;
        normalized = normalized - Math.floor(normalized); // wrap into [0, 1)

        double ticks = normalized * ENCODER_TICKS_PER_REV;
        if (ENCODER_INVERTED) {
            ticks = ENCODER_TICKS_PER_REV - ticks;
        }

        ticks = (ticks - ENCODER_ZERO_TICKS) % ENCODER_TICKS_PER_REV;
        if (ticks < 0) ticks += ENCODER_TICKS_PER_REV;

        return ticks;
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
        detectedLocation = -1;
        detectioncnt=0;
        timer.reset();
        lastDetected=false;
        waitingCSIntegration = false;
        intakeTimer.reset();
        spindexerPID.init();
    }

    public boolean spinToMotif(int motifIndex) {
        int nextmotif = dummyMotif[motifIndex];
        if (outtakesensor.getDetected() != nextmotif && !lastDetected) {
            spindexerServo.setPower(0.75);
            detectedLocation = -1;
            lastDetected=false;
        } else if (detectedLocation==-1){
            /*TODO:Adjust for color sensor offset
            If adjust is negative,
               detectedLocation = Math.abs(spindexerSensor.getCurrentPosition()-adjust);
            If adjust is positive,
               detectedLocation = (spindexerSensor.getCurrentPosition()+adjust)%3.3;
            */
            detectedLocation = Math.abs(getSpindexerTicks() - adjust);
            lastDetected=true;
            spindexerPID.init();
        }
        if (detectedLocation != -1) {
            double pos = getSpindexerTicks();
            spindexerServo.setPower(spindexerPID.update(calculateError(detectedLocation, pos)));
            return pos >= detectedLocation - autoSpinEpsilon && pos <= detectedLocation + autoSpinEpsilon;
        }
        return false;
    }

    public boolean spinToMotifV2(int motifIndex) {
        if (detectioncnt==-1) detectioncnt=0;
        if (detectioncnt==3) return false;
        int nextmotif = dummyMotif[motifIndex];
        double pos = getSpindexerTicks();
        if (Math.abs(calculateError(outslotsV[currentSlot], pos))>autoSpinEpsilon && !waitingCSIntegration){
            spindexerServo.setPower(spindexerPID.update(calculateError(outslotsV[currentSlot], pos)));
        }else{
            if (!waitingCSIntegration){
                waitingCSIntegration=true;
                intakeTimer.reset();
            }
            holdSpindexer();
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
     * @return true if successful, false if still searching or no empty slot found
     */
    public boolean spinToIntake() {
        if (detectioncnt==-1){
            detectioncnt=0;
        }
        if (detectioncnt==3){
            spindexerServo.setPower(0);
            return false;
        }
        double pos = getSpindexerTicks();
        if (Math.abs(calculateError(inslotsV[currentSlot], pos))>autoSpinEpsilon && !waitingCSIntegration){
            spindexerServo.setPower(Math.min(spindexerPID.update(calculateError(inslotsV[currentSlot], pos)),0.75));
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
                if (spindexerSlots[i]==(int) Math.round(getSpindexerTicks())){
                    spindexerSlots[i]=1;
                }
            }
            return numBalls;
        }
        return 0;
    }

    public void holdSpindexer(){
        double pos = getSpindexerTicks();
        spindexerServo.setPower(Math.min(spindexerPID.update(calculateError(inslotsV[currentSlot], pos)),0.75));
    }

    public boolean spinAfterIntake(boolean intakesuccess){
        double epsilon =0.01;
        if (intakesuccess){
            spindexerServo.setPower(0.75);
        }else{
            detectedLocation=getSpindexerTicks();
            spindexerPID.init();
        }
        if (detectedLocation!=-1){
            double pos = getSpindexerTicks();
            spindexerServo.setPower(spindexerPID.update(detectedLocation - pos));
            return pos>=detectedLocation-epsilon&&pos<=detectedLocation+epsilon;
        }
        return false;
    }
}
