//package org.firstinspires.ftc.teamcode.Stanley.testingOpmodes;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.CRServo;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.PID;
//import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtakeV2;
//import org.firstinspires.ftc.teamcode.Stanley.finalizedClasses.outtakeV3;
//
//@TeleOp
//public class tuningTurret extends LinearOpMode {
//    CRServo turret =null;
//    outtakeV3 outtakeOperator=null;
//    double change=0.1;
//    int x=0;
//    //TODO:Get real value+sync with outtakeV2 value
//    //test
//    double angle=60;
//    boolean correctingtoggle=false;
//    boolean firsttime=true;
//    //FTC dashboard telemetry
//    FtcDashboard dashboard=null;
//    Telemetry dashboardTelemetry=null;
//
//    @Override
//    public void runOpMode(){
//        turret =hardwareMap.get(CRServo.class,"turretServo");
//        outtakeOperator=new outtakeV3(hardwareMap,"Red",true,null);
//        dashboard = FtcDashboard.getInstance();
//        dashboardTelemetry = dashboard.getTelemetry();
//        waitForStart();
//        while (opModeIsActive()){
//            if (gamepad1.yWasPressed()) correctingtoggle=!correctingtoggle;
////            if (gamepad1.aWasPressed()) outtakeOperator.initHoodAngleBlocking();
//            //shift speed
//            if (gamepad1.rightBumperWasPressed()){
//                change*=10;
//            }else if (gamepad1.leftBumperWasPressed()){
//                change/=10;
//            }
//            telemetry.addData("Change",change);
//            //selection
//            if (gamepad1.dpadLeftWasPressed()){
//                x--;
//                if (x<0){
//                    x=2;
//                }
//            }
//            if (gamepad1.dpadRightWasPressed()){
//                x++;
//                if (x>2){
//                    x=0;
//                }
//            }
//            if (gamepad1.dpadUpWasPressed()){
//                outtakeOperator.Kturn[x]+=change;
//            }
//            if (gamepad1.dpadDownWasPressed()){
//                outtakeOperator.Kturn[x]-=change;
//            }
////            if (gamepad1.leftStickButtonWasPressed()){
////                angle+=change;
////            }
////            if (gamepad1.rightStickButtonWasPressed()){
////                angle-=change;
////            }
//            String line1="Kturn: ";
//            for (int i=0;i<=2;i++){
//                if (i==x){
//                    line1+="{";
//                }
//                outtakeOperator.Kturn[i]=(double) Math.round(outtakeOperator.Kturn[i] * Math.pow(10, 7)) / Math.pow(10, 7);
//                line1+=outtakeOperator.Kturn[i];
//                if (i==x){
//                    line1+="}";
//                }
//                line1+=", ";
//            }
//            if (gamepad1.xWasPressed()){
//                outtakeOperator.turnPID=new PID(outtakeOperator.Kturn[0],outtakeOperator.Kturn[1],outtakeOperator.Kturn[2]);
//            }
//            boolean atTarget=false;
//            if (correctingtoggle){
//                if (firsttime){
//                    outtakeOperator.turnPID.init();
//                    firsttime=false;
//                }
//                atTarget=outtakeOperator.autoturn();
//            }else{
//                turret.setPower(0);
//                firsttime=true;
//            }
////            outtakeOperator.updateHoodAngle();
//
//            telemetry.addLine(line1);
//            telemetry.addData("Holding",correctingtoggle);
////            telemetry.addData("Target",angle);
//            telemetry.addData("Offset(deg)",outtakeOperator.apriltag.getYaw());
////            telemetry.addData("CurrentV",outtakeOperator.hoodSensor.getVoltage());
//            telemetry.addData("Power",outtakeOperator.turnPID.power);
////            telemetry.addData("Offset(deg)",angle/outtakeOperator.servoDegPerRot-outtakeOperator.hoodAngle);
////            telemetry.addData("AtTarget",atTarget);
//            telemetry.update();
//            dashboardTelemetry.addLine(line1);
//            dashboardTelemetry.addData("Holding",correctingtoggle);
////            dashboardTelemetry.addData("Target",angle);
//
//            dashboardTelemetry.addData("Offset(deg)",outtakeOperator.apriltag.getYaw());
////            dashboardTelemetry.addData("CurrentV",outtakeOperator.hoodSensor.getVoltage());
//            dashboardTelemetry.addData("Power",outtakeOperator.turnPID.power);
////            dashboardTelemetry.addData("Offset(deg)",angle/outtakeOperator.servoDegPerRot-outtakeOperator.hoodAngle);
////            dashboardTelemetry.addData("AtTarget",atTarget);
//            dashboardTelemetry.update();
//        }
//    }
//}
