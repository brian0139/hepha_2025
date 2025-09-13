package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="servotest")
public class actionservotest extends LinearOpMode {
    @Override
    public void runOpMode(){
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Servo servo=hardwareMap.servo.get("servoSpeed");
        TouchSensor touchS=hardwareMap.touchSensor.get("touch");

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0,0,90))
                        .lineToY(20)
                        .stopAndAdd(new servomove(servo,touchS,1,true))
                        .lineToY(0)
                        .stopAndAdd(new servomove(servo,touchS,0,false))
                        .build()
        );
    }
    public class servomove implements Action {
        Servo servo;
        TouchSensor touchS;
        double position;
        boolean usetouch;
        ElapsedTime timer;
        boolean pressed;

        public servomove(Servo s,TouchSensor touchS, double p,boolean touch){
            this.servo=s;
            this.touchS=touchS;
            this.position=p;
            this.usetouch=touch;
            this.pressed=false;
        }

        @Override
        public boolean run(TelemetryPacket telemetryPacket){
            if (timer==null && !usetouch) {
                timer = new ElapsedTime();
                servo.setPosition(position);
                telemetry.addLine("autotriggered");
                telemetry.update();
            }
            if (!usetouch) {
                telemetry.addData("Time:",timer.seconds());
                telemetry.update();
                return timer.seconds() < 2;
            }
            if (usetouch){
                if (pressed) {
                    return timer.seconds() < 3;
                }
                if (touchS.isPressed()) {
                    telemetry.addLine("manual");
                    telemetry.addData("pressed",pressed);
                    telemetry.update();
                    servo.setPosition(position);
                    timer = new ElapsedTime();
                    pressed = true;
                    return true;
                } else {
                    return true;
                }
            }
            return false;
        }
    }
}