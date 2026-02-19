package org.firstinspires.ftc.teamcode.Stanley.finalizedClasses;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Aaron.aprilTagV3;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

public class outtakeV3FittedAutolaunch {
    //Team color
    String teamColor;

    //================================  April Tag  ================================
    //April tag processor
    public aprilTagV3 apriltag;

    //================================  Flywheel  ================================
    //Outtake flywheel
    public DcMotorEx flywheelDriveR;
    public DcMotorEx flywheelDrive;

    //================================  Transfer  ================================
    //transfer servo
    public DcMotor transfer;
    //transfer positions(up, down)
    public static double[] transferpowers ={1,0};

    //================================  Hood  ================================
    //Motor for hood encoder
    public DcMotor hoodEncoder=null;
    //Outtake Hood Servo
    public CRServo hoodServo;
    public CRServo turretServo;
    //Degrees changed for every servo rotation
    public double servoDegPerRot =24.18;
    //Ticks/revolution for encoder
    public int ticksPerRevHood=8192;
    //PID instance for hood
//    public double[] Kh={0.0005,0.0005,0.00003};
    //35 little too low
    public double[] Kh={0.0004,0.0005,0.00001};
    public PIDhood hoodPID=new PIDhood(Kh[0],Kh[1],Kh[2]);

    //================================  Turret  ================================
    //Robot drivetrain object
    MecanumDrive drive = null;
    //Acceptable angle rotation range of turret(0=right, 90=up, 180=left, 270=down)
    public double minTurretAngle=0;
    public double maxTurretAngle=180;
    //Turret autoaim epsilon
    public double turretEpsilon=2.5;
    //auto aim vars
    // P, I, D
//    public double[] Kturn={0.0072,0.004,0.0023};
    public double[] Kturn={0.0135,0.0053,0.0013};
    public PID turnPID=new PID(Kturn[0],Kturn[1],Kturn[2]);

    //================================  Config  ================================
    //target april tag id
    int targetTagID=-1;
    //to use turret rotation limitation or not(requires initialized MecanumDrive
    public boolean setRange=false;
    double[] distances     = new double[]{2,3,4,5,6,7,9,11};
    double[] hoodDegrees   = new double[]{37.19,36.69,37.34,39.71,43.14,40.01,43.2,48.75};
    double[] flywheelSpeeds= new double[]{1800,1430,1470,1620,1660,1730,1960,2100};

    /**
     * Constructor
     * @param hardwareMap Hardwaremap
     * @param teamColor Team color("Red" or "Blue")
     * @param useTag To use april tag process or not
     * @param drive Robot drivetrain
     */
    public outtakeV3FittedAutolaunch(HardwareMap hardwareMap, String teamColor, boolean useTag, MecanumDrive drive){
        this.flywheelDriveR = hardwareMap.get(DcMotorEx.class,"flywheelR");
        this.flywheelDrive=hardwareMap.get(DcMotorEx.class,"flywheel");
        flywheelDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        this.drive=drive;
        this.teamColor=teamColor;
        this.hoodServo=hardwareMap.get(CRServo.class,"hoodServo");
        this.turretServo =hardwareMap.get(CRServo.class,"turretServo");
        this.hoodEncoder=hardwareMap.get(DcMotorEx.class,"leftBack");
        this.transfer=hardwareMap.get(DcMotor.class,"par1");
        hoodPID.maxOutput=1;
        hoodPID.minOutput=-1;
        //set target april tag number to aim at depending on team color.
        if (Objects.equals(this.teamColor, "Red") && this.targetTagID!=-1){
            this.targetTagID=24;
        }
        else if (Objects.equals(this.teamColor, "Blue") && this.targetTagID!=-1) {
            this.targetTagID = 20;
        }
        //Init apriltag instance
        if (useTag) {
            this.apriltag = new aprilTagV3(hardwareMap);
            this.apriltag.setPipeline(5);
            this.apriltag.init();
        }
    }

    /**
     * Helper function to normalize angle to +-180 range
     * @param angle
     * @return
     */
    public static double normalizeAngle(double angle) {
        angle = angle % 360.0;
        if (angle > 180.0) {
            angle -= 360.0;
        } else if (angle < -180.0) {
            angle += 360.0;
        }
        return angle;
    }

    /**
     * Autoaim to april tag.
     * Target tag set by teamColor variable in class, "Red" or "Blue"
     * @return False if canceled or teamColor not found, True if successful
     */
    public boolean autoturn(){
        this.apriltag.scanOnce();
        if (!apriltag.hasValidTarget()){
            turretServo.setPower(0);
            turnPID=new PID(Kturn[0],Kturn[1],Kturn[2]);
            if (setRange) {
                //get current heading
                double currentHeading = drive.localizer.getPose().heading.toDouble();
                //Shift heading from 180 to -180 to 0 to 360
                if (currentHeading < 0) {
                    currentHeading += 360;
                }
                //get current position
                Vector2d currentPos = drive.localizer.getPose().position;
                // Robot position
                double xr = currentPos.x;
                double yr = currentPos.y;

                // Target position
                double xt = -61.5;
                double yt = 0;
                if (Objects.equals(this.teamColor, "Red")) {
                    yt = 52.5;
                } else if (Objects.equals(this.teamColor, "Blue")) {
                    yt = -53.5;
                }

                // Robot facing angle in degrees (-180 to 180)
                double robotAngle = currentHeading;

                // Compute vector to target
                double dx = xt - xr;
                double dy = yt - yr;

                // Target angle in world coordinates (-180 to 180)
                double targetAngle = Math.toDegrees(Math.atan2(dy, dx));

                // Angle of target relative to robot front
                double relativeAngle = normalizeAngle(targetAngle - robotAngle);

                // Attempt to move camera within range
                if (relativeAngle > maxTurretAngle) {
                    turretServo.setPower(0.1);
                } else if (relativeAngle < minTurretAngle) {
                    turretServo.setPower(-0.1);
                }
            }
            return false;
        }
        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double  headingError    = apriltag.getYaw();

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        double power   = turnPID.update(headingError) ;
        turretServo.setPower(power);
        return Math.abs(apriltag.getYaw())<=turretEpsilon;
    }

//    /**
//     * Get distance to april tag using limelight
//     * @return Distance in in.
//     */
//    public double getDistance(){
//        /*
//          Formula:
//          d=(h2-h1)/tan (a1+a2)
//          h1: Height of the Limelight lens from the floor.
//          h2: Height of the target (AprilTag) from the floor.
//          a1: Mounting angle of the Limelight (degrees back from vertical).
//          a2: Vertical offset angle to the target, obtained from the Limelight's ty value.
//         */
//        return (29.5-11.4375)/Math.tan(Math.toRadians(20-apriltag.getPitch()));
////        return (18.75-11.4375)/Math.tan(Math.toRadians(20-apriltag.getPitch()));
//    }
    /**
     * Get april tag size using limelight
     * @return Distance in in.
     */
    public double getDistance(){
        return this.apriltag.getTargetArea();
    }

    public void setPipeLine(int pipeline){
        apriltag.setPipeline(pipeline);
    }
    /**
     * Find optimal launch angle and velocity to hit a target with specific impact angle constraints.
     *
     * @param distance Distance to target in in.
     * @return Map<string,string> with keys: angle, velocity(in encoder ticks/s)
     */
    public Map<String,String> findOptimalLaunch(double distance) {
        if (distance < 25.2){
            return new HashMap<>(Map.of(
                    "angle", Double.toString(-1.0),
                    "velocity", Double.toString(-1.0)
            ));
        }else{
            double velocity;
            double angle;
            if (distance<=60.2){
                velocity = (0.001292 * Math.pow(distance, 4)) + (-0.202462 * Math.pow(distance, 3)) + (11.418102 * Math.pow(distance, 2)) + (-267.783745 * distance) + (3791.182542);
            }else{
                velocity=(0.002871 * Math.pow(distance, 3)) + (-0.9114 * Math.pow(distance, 2)) + (98.260385 * distance) + (-1390.717691);
            }
            if (distance<=60.2){
                angle = (-0.002594 * Math.pow(distance, 4)) + (0.179021 * Math.pow(distance, 3)) + (7.163897 * Math.pow(distance, 2)) + (-759.400699 * distance) + (19466.886344);
            }else{
                angle = (0.019954 * Math.pow(distance, 3)) + (-3.402398 * Math.pow(distance, 2)) + (101.682974 * distance) + (6552.759184);
            }
            return new HashMap<>(Map.of(
                    "angle", Double.toString(angle),
                    "velocity", Double.toString(velocity)
            ));
        }
    }

    /**
     * Set the hood angle to a specific degree
     * @param degrees degrees from hood to horizontal(cnt. clockwise)
     * @return if hood is at position
     */
    public boolean setHood(double degrees){
        double epsilon=40;
        double targetTicks=(66.81-degrees)/servoDegPerRot*ticksPerRevHood;
        double power=hoodPID.update(targetTicks-hoodEncoder.getCurrentPosition());
        hoodServo.setPower(-power);
        return (hoodEncoder.getCurrentPosition() >= targetTicks - epsilon) && (hoodEncoder.getCurrentPosition() <= targetTicks + epsilon);
    }

    /**
     * Set the hood angle to a specific degree
     * @param encoderTicks degrees from hood to horizontal(cnt. clockwise)
     * @return if hood is at position
     */
    public boolean setHoodEncoder(double encoderTicks){
        double epsilon=40;
        double power=hoodPID.update(encoderTicks-hoodEncoder.getCurrentPosition());
        hoodServo.setPower(-power);
        return (hoodEncoder.getCurrentPosition() >= encoderTicks - epsilon) && (hoodEncoder.getCurrentPosition() <= encoderTicks + epsilon);
    }

    /**
     * Auto-initializes hood(spin off highest, then reset angle counter)
     * WARNING:Blocking
     */
    public void initHoodAngleBlocking(){
        hoodEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        while((hoodEncoder.getCurrentPosition())> -3*ticksPerRevHood) hoodServo.setPower(1);
        hoodServo.setPower(0);
        hoodEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hoodEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Resets hood angle counter to highest(gear off)(AKA 0)
     * Will cause left-back drivetrain motor to temporarily lose power,
     * FOR USE IN EMERGENCIES ONLY
     */
    public void resetHoodAngle(){
        hoodEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hoodEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Stops hood servo
     */
    public void stopHood(){
        hoodServo.setPower(0);
    }

    /**
     * Transfer artifact to flywheel(move transfer up)
     */
    public void transferUp(){this.transfer.setPower(transferpowers[0]);}
    /**
     * Lower Transfer
     */
    public void transferDown(){
        this.transfer.setPower(transferpowers[1]);
    }

    /**
     * Spin flywheel to speed
     * @param targetSpeed Target flywheel speed in encoder ticks/sec.
     * @param tolerance Tolerance in ticks/sec. from target speed to return true.
     * @return If flywheel is up to speed.
     */
    public boolean spin_flywheel(double targetSpeed, int tolerance){
        DcMotorEx flywheelDriveEx=this.flywheelDriveR;
        flywheelDriveEx.setVelocity(targetSpeed);
        flywheelDrive.setVelocity(targetSpeed);
        return targetSpeed - tolerance <= flywheelDriveEx.getVelocity() && flywheelDriveEx.getVelocity() <= targetSpeed + tolerance;
    }
}
