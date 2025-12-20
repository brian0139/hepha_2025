package org.firstinspires.ftc.teamcode.Brian;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class spindexerTestMotif extends LinearOpMode {
    public boolean full=false;
    public int canOuttake=0;
    public int[] motifPattern={1,1,2}; //ggp
    public int motifIndex=0;
    private Servo spindexerServo=null;
    double[] outtakeslots = {0.65,1,0.26};
    double[] intakeslots = {0.05,0.44,0.83};
    int outtakepos=0;
    int intakepos=0;

    @Override
    public void runOpMode() throws InterruptedException {
        spindexerServo=hardwareMap.get(Servo.class,"spindexerServo");
        spindexer spindexerObject = new spindexer(spindexerServo);
        waitForStart();
        while (opModeIsActive()){
            if (spindexerObject.spindexerSlots[0]!=0 && spindexerObject.spindexerSlots[1]!=0 && spindexerObject.spindexerSlots[3]!=0){
                full=true;
            }
            if (full){
                for (int i=0;i<3;i++){
                    if (spindexerObject.spindexerSlots[i]==motifPattern[motifIndex]){
                        spindexerObject.rotateSpindexerOutput(i);
                    } else {
                        canOuttake++;
                    }
                }
                if (canOuttake==3){
                    for (int i=0;i<3;i++){
                        spindexerObject.rotateSpindexerInput(i);

                        
                    }
                }

            }

        }
    }
}
