package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Brian.spindexerColor;
import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.PID;

@TeleOp
public class spindexerpausetuning extends LinearOpMode {
    CRServo hood=null;
    DcMotor hoodSensor=null;
    spindexerColor outtakeOperator=null;
    double change=0.1;
    int x=0;
    double[] values={300,500,0.35};
    //TODO:Get real value+sync with outtakeV2 value
    //test
    double angle=66.81;
    boolean correctingtoggle=false;
    //FTC dashboard telemetry
    FtcDashboard dashboard=null;
    Telemetry dashboardTelemetry=null;

    @Override
    public void runOpMode(){
        hood=hardwareMap.get(CRServo.class,"spindexerServo");
        hoodSensor=hardwareMap.get(DcMotor.class,"intake");
        outtakeOperator=new spindexerColor(hood,hoodSensor,hardwareMap);
        SpinToIntake spin=new SpinToIntake(0,outtakeOperator);
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.yWasPressed()) correctingtoggle=!correctingtoggle;
            if (gamepad1.aWasPressed()) {
                outtakeOperator=new spindexerColor(hood,hoodSensor,hardwareMap);
                outtakeOperator.mindetectiontime=values[0];
                outtakeOperator.maxdetectiontime=values[1];
                outtakeOperator.spinspeed=values[2];
                spin=new SpinToIntake(0,outtakeOperator);
            }
            //shift speed
            if (gamepad1.rightBumperWasPressed()){
                change*=10;
            }else if (gamepad1.leftBumperWasPressed()){
                change/=10;
            }
            telemetry.addData("Change",change);
            //selection
            if (gamepad1.dpadLeftWasPressed()){
                x--;
                if (x<0){
                    x=2;
                }
            }
            if (gamepad1.dpadRightWasPressed()){
                x++;
                if (x>2){
                    x=0;
                }
            }
            if (gamepad1.dpadUpWasPressed()){
                values[x]+=change;
            }
            if (gamepad1.dpadDownWasPressed()){
                values[x]-=change;
            }
            String line1="Values: ";
            for (int i=0;i<=2;i++){
                if (i==x){
                    line1+="{";
                }
                values[i]=(double) Math.round(values[i] * Math.pow(10, 5)) / Math.pow(10, 5);
                line1+=values[i];
                if (i==x){
                    line1+="}";
                }
                line1+=", ";
            }
            if (correctingtoggle){
                spin.run();
            }else{
                spin.spindexer.spindexerServo.setPower(0);
            }

            telemetry.addLine(line1);
            telemetry.update();
        }
    }
    /**
     * Spins spindexer to next empty slot for intake
     */
    class SpinToIntake{
        private final int motifIndex;
        private boolean initialized = false;
        private spindexerColor spindexer=null;

        public SpinToIntake(int motifIndex,spindexerColor spindexer) {
            this.motifIndex = motifIndex;
            this.spindexer=spindexer;
        }

        public boolean run() {
            if (!initialized) {
                spindexer.initSpin();
                initialized = true;
            }

            boolean complete = spindexer.spinToIntake();

            telemetry.addData("Spindexer: Motif Index", motifIndex);
            telemetry.addData("Spindexer: Status", complete ? "Complete" : "Spinning");

            return !complete; // Return false when complete
        }
    }
}
