package org.firstinspires.ftc.teamcode.Stanley.finalizedClasses;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Aaron.aprilTagV3;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

public class outtakeV3 {
    //Team color
    String teamColor;
    //April tag processor
    public aprilTagV3 apriltag;
    //Outtake flywheel
    public DcMotorEx flywheelDriveR;
    public DcMotorEx flywheelDrive;
    //Outtake Hood Servo
    CRServo hoodServo;
    CRServo turretServo;
    public DcMotorEx hoodSensor;
    //Degrees changed for every servo rotation
    public double servoDegPerRot =26.53;
    //Ticks/revolution for encoder
    public int ticksPerRevHood=8192;
    //hood Axon voltage last loop
    double lastVolt=-1;
    //# of rotations hood servo has
    int rotations=0;
    //transfer positions(up, down)
    public static double[] transferpowers ={0.5,0};
    //reference rotations
    public double referenceRotation=-1;
    //hood angle(in degrees)
    public double hoodAngle=-1;
    //transfer servo
    public DcMotor transfer;
    //drivetrain motors
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    //auto aim vars
    //  Drive = Error * Gain
    // Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    public double[] Kturn={0.01,0.1,0.007};
    public PID turnPID=new PID(Kturn[0],Kturn[1],Kturn[2]);
    //vars
    int targetTagID=-1;
    //voltage jump to be considered a rotation
    double maxVJump=3.3*0.5;
    //PID instance for hood
    public double[] Kh={3.0,0,0.06};
    public PID hoodPID=new PID(Kh[0],Kh[1],Kh[2]);
    public outtakeV3(HardwareMap hardwareMap, DcMotorEx flywheelDrive, DcMotorEx flywheelDriveR, String teamColor, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, CRServo hoodServo, DcMotorEx hoodSensor, DcMotor transfer, boolean useTag){
        this.flywheelDriveR = flywheelDriveR;
        this.flywheelDrive=flywheelDrive;
        this.teamColor=teamColor;
        this.leftFront=leftFront;
        this.rightFront=rightFront;
        this.leftBack=leftBack;
        this.rightBack=rightBack;
        this.hoodServo=hoodServo;
        this.turretServo =hardwareMap.get(CRServo.class,"turretServo");
        this.hoodSensor=hoodSensor;
        this.transfer=transfer;
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
     * Autoaim to april tag.
     * Target tag set by teamColor variable in class, "Red" or "Blue"
     * @return False if canceled or teamColor not found, True if successful
     */
    public boolean autoturn(){
        this.apriltag.scanOnce();
        if (!apriltag.hasValidTarget()){
            turretServo.setPower(0);
            return false;
        }
        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double  headingError    = apriltag.getYaw();

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        double power   = turnPID.update(headingError) ;
        turretServo.setPower(power);
        return true;
    }

    public void setPipeLine(int pipeline){
        apriltag.setPipeline(pipeline);
    }
    /**
     * Find optimal launch angle and velocity to hit a target with specific impact angle constraints.
     * Theoretical launch velocity calculation:
     * PI*Diameter*RPM/60/2,
     * /2 only for single flywheel.
     * Note this value is assuming perfect conditions with 0 friction & slippage, actual values may be lower.
     *
     * @param targetX horizontal distance to target (inches)
     * @param targetY vertical height of target above launch point (inches)
     * @param minAngleDeg minimum hood angle range (degrees from horizontal)
     * @param maxAngleDeg maximum hood angle range (degrees from horizontal)
     * @param maxVelocity maximum launch velocity (inches/second)
     * @param minImpactAngleDeg minimum acceptable impact angle (degrees from horizontal, 180=straight down)
     * @param maxImpactAngleDeg maximum acceptable impact angle (degrees from horizontal, 180=straight down)
     * @param g gravitational acceleration (inches/s^2, default 386.4 for Earth)
     * @param yEpsilon acceptable y-position error for hitting target (inches)
     * @param angleEpsilon convergence threshold for angle binary search (degrees)
     * @param maxIterations maximum iterations before giving up
     * @return Map<string,string> with keys: angle, velocity, impactAngle, yError, impactAngleError, success, reason
     */
    public static Map<String,String> findOptimalLaunch(double targetX, double targetY,
                                                       double minAngleDeg, double maxAngleDeg,
                                                       double maxVelocity,
                                                       double minImpactAngleDeg, double maxImpactAngleDeg,
                                                       double g, double yEpsilon, double angleEpsilon,
                                                       int maxIterations) {

        // Convert angles to radians for calculations
        double minAngle = Math.toRadians(minAngleDeg);
        double maxAngle = Math.toRadians(maxAngleDeg);
        double minImpactAngle = Math.toRadians(minImpactAngleDeg);
        double maxImpactAngle = Math.toRadians(maxImpactAngleDeg);
        double medianImpactAngle = (minImpactAngle + maxImpactAngle) / 2.0;

        // Track best solution found so far
        Map<String,String> bestSolution = null;
        double bestDistanceToMedian = Double.POSITIVE_INFINITY;

        // Binary search on launch angle
        double lowAngle = minAngle;
        double highAngle = maxAngle;

        String reason = "";

        for (int iteration = 0; iteration < maxIterations; iteration++) {
            // Check convergence: if angle range is very small, we've converged
            if ((highAngle - lowAngle) < Math.toRadians(angleEpsilon)) {
                reason = "converged";
                break;
            }

            double midAngle = (lowAngle + highAngle) / 2.0;

            // Binary search on velocity for this angle
            double foundVelocity = -1;
            double lowVelocity = 0;
            double highVelocity = maxVelocity;

            for (int velIteration = 0; velIteration < 100; velIteration++) {
                double midVelocity = (lowVelocity + highVelocity) / 2.0;

                // Calculate y position at target x using projectile motion equation
                // y = x*tan(θ) - (g*x²)/(2*v²*cos²(θ))
                double yAtTarget;
                try {
                    double cosAngle = Math.cos(midAngle);
                    if (midVelocity == 0 || cosAngle == 0) {
                        // Velocity too low, causes numerical issues
                        lowVelocity = midVelocity;
                        continue;
                    }
                    yAtTarget = targetX * Math.tan(midAngle) -
                            (g * targetX * targetX) / (2.0 * midVelocity * midVelocity * cosAngle * cosAngle);
                } catch (Exception e) {
                    // Velocity too low, causes numerical issues
                    lowVelocity = midVelocity;
                    continue;
                }

                // Check if we hit the target within epsilon
                if (Math.abs(yAtTarget - targetY) < yEpsilon) {
                    foundVelocity = midVelocity;
                    break;
                } else if (yAtTarget < targetY) {
                    // Undershooting, need more velocity
                    lowVelocity = midVelocity;
                } else {
                    // Overshooting, need less velocity
                    highVelocity = midVelocity;
                }
            }

            // Case 1: No valid velocity found (even max velocity undershoots)
            if (foundVelocity == -1) {
                // At max velocity, check if we're undershooting or can't reach
                double cosAngle = Math.cos(midAngle);
                double yAtTargetMaxVel = targetX * Math.tan(midAngle) -
                        (g * targetX * targetX) / (2.0 * maxVelocity * maxVelocity * cosAngle * cosAngle);

                if (yAtTargetMaxVel < targetY) {
                    // We're undershooting - could be angle too low OR too high
                    // Check if we can even reach target_x horizontally (max range formula)
                    double maxRange = (maxVelocity * maxVelocity * Math.sin(2.0 * midAngle)) / g;

                    if (maxRange < targetX) {
                        // Can't reach target horizontally - angle too high (too steep)
                        highAngle = midAngle;
                    } else {
                        // Can reach horizontally but undershooting vertically
                        // Need to check vertex position to determine if angle is too high or too low
                        double tVertex = maxVelocity * Math.sin(midAngle) / g;
                        double vertexX = maxVelocity * Math.cos(midAngle) * tVertex;

                        if (vertexX < targetX) {
                            // Vertex before target - on descending part of parabola
                            // Undershooting means angle is too high (falling too steeply)
                            highAngle = midAngle;
                        } else {
                            // Vertex at/after target - on ascending part of parabola
                            // Undershooting means angle is too low (not enough height)
                            lowAngle = midAngle;
                        }
                    }
                }
            }
            // Case 2: Valid velocity found, check impact angle
            else {
                // Calculate impact angle (angle of velocity vector at target)
                // v_x = v*cos(θ) (constant)
                // v_y = v*sin(θ) - g*t
                // At target x: t = x/(v*cos(θ))
                double tImpact = targetX / (foundVelocity * Math.cos(midAngle));
                double vX = foundVelocity * Math.cos(midAngle);
                double vY = foundVelocity * Math.sin(midAngle) - g * tImpact;

                // Impact angle from horizontal (negative v_y means going down)
                // atan2 gives angle from positive x-axis
                double impactAngle = Math.atan2(vY, vX);
                // Convert to 0-180 range where 180 is straight down
                if (impactAngle < 0) {
                    impactAngle = Math.PI + impactAngle;  // Convert negative angles
                }

                // Check if impact angle is within acceptable range
                if (minImpactAngle <= impactAngle && impactAngle <= maxImpactAngle) {
                    // Valid solution! Check if it's closer to median than previous best
                    double distanceToMedian = Math.abs(impactAngle - medianImpactAngle);
                    // Calculate y position at target x using projectile motion equation
                    // y = x*tan(θ) - (g*x²)/(2*v²*cos²(θ))
                    double yAtTarget;
                    double cosAngle = Math.cos(midAngle);
                    yAtTarget = targetX * Math.tan(midAngle) -
                            (g * targetX * targetX) / (2.0 * foundVelocity * foundVelocity * cosAngle * cosAngle);
                    if (distanceToMedian <= bestDistanceToMedian) {
                        bestDistanceToMedian = distanceToMedian;
                        bestSolution = new HashMap<>(Map.of(
                                "angle",Double.toString(Math.toDegrees(midAngle)),
                                "velocity",Double.toString(foundVelocity),
                                "impactAngle",Double.toString(Math.toDegrees(impactAngle)),
                                "yError",Double.toString(Math.abs(targetY-yAtTarget)), // Within epsilon by definition
                                "impactAngleError",Double.toString(distanceToMedian), // Within range by definition
                                "success","true",
                                "reason","valid_solution"
                        ));
                    }

                    // Update angle search based on impact angle relative to median
                    if (impactAngle < medianImpactAngle) {
                        // Impact too shallow, launch angle too low
                        highAngle = midAngle;
                    } else {
                        // Impact too steep, launch angle too high
                        lowAngle = midAngle;
                    }
                } else {
                    // Impact angle outside acceptable range
                    if (impactAngle<=Math.PI/2){
                        //On ascending part of parabola, launch angle too low
                        lowAngle=midAngle;
                    }
                    else if (impactAngle > maxImpactAngle) {
                        // Impact too steep, launch angle too high
                        highAngle = midAngle;
                    } else {
                        // Impact too shallow, launch angle too low
                        lowAngle = midAngle;
                    }
                    // Calculate y position at target x using projectile motion equation
                    // y = x*tan(θ) - (g*x²)/(2*v²*cos²(θ))
                    double yAtTarget;
                    double cosAngle = Math.cos(midAngle);
                    yAtTarget = targetX * Math.tan(midAngle) -
                            (g * targetX * targetX) / (2.0 * foundVelocity * foundVelocity * cosAngle * cosAngle);
                    // Store as backup solution if we don't find anything better
                    double impactAngleError = Math.min(Math.abs(impactAngle - minImpactAngle),
                            Math.abs(impactAngle - maxImpactAngle));
                    if (bestSolution == null || impactAngleError < Double.parseDouble(bestSolution.get("impactAngleError"))) {
                        bestSolution = new HashMap<>(Map.of(
                                "angle",Double.toString(Math.toDegrees(midAngle)),
                                "velocity",Double.toString(foundVelocity),
                                "impactAngle",Double.toString(Math.toDegrees(impactAngle)),
                                "yError",Double.toString(Math.abs(targetY-yAtTarget)), // Within epsilon by definition
                                "impactAngleError",Double.toString(impactAngleError), // Within range by definition
                                "success","false",
                                "reason","impact_angle_out_of_range"
                        ));
                    }
                }
            }
        }

        // If loop completed without convergence
        if (reason.isEmpty()) {
            reason = "max_iterations";
        }

        // If we found a valid solution, return it
        if (bestSolution != null && Boolean.parseBoolean(bestSolution.get("success"))) {
            return bestSolution;
        }

        // Otherwise, return best effort
        if (bestSolution == null) {
            // No solution attempted velocity at all
            bestSolution = new HashMap<>(Map.of(
                    "angle",Double.toString(-1.0),
                    "velocity",Double.toString(-1.0),
                    "impactAngle",Double.toString(-1.0),
                    "yError",Double.toString(Double.POSITIVE_INFINITY), // Within epsilon by definition
                    "impactAngleError",Double.toString(Double.POSITIVE_INFINITY), // Within range by definition
                    "success","false",
                    "reason","no_solution_found"
            ));
        }

        return bestSolution;
    }

    /**
     * Set the hood angle to a specific degree
     * @param degrees degrees from hood to horizontal(cnt. clockwise)
     * @return if hood is at position
     */
    public boolean setHood(double degrees){
        double epsilon=0.001;
        double targetRotations=degrees/servoDegPerRot;
        double power=hoodPID.update(targetRotations-hoodAngle);
        hoodServo.setPower(power);
        updateHoodAngle();
        return hoodAngle >= targetRotations - epsilon && hoodAngle <= targetRotations + epsilon;
    }

    /**
     * Auto-initializes hood(spin off highest, then reset angle counter)
     * WARNING:Blocking
     */
    public void initHoodAngleBlocking(){
        for (int i=0;i<=3;i++) {
            while (hoodSensor.getVoltage() >= 0.2) hoodServo.setPower(1);
            while (hoodSensor.getVoltage() < 0.2) hoodServo.setPower(1);
        }
        hoodAngle=66.81;
    }

    /**
     * Resets hood angle counter to highest(gear off)
     * FOR USE IN EMERGENCIES ONLY
     */
    public void resetHoodAngle(){
        hoodAngle=66.81;
    }

    public void stopHood(){
        hoodServo.setPower(0);
        updateHoodAngle();
    }

    /**
     * Helper function to update current angle of hood
     * @return Delta change in hood angle(in servo rotations)
     */
    public double updateHoodAngle(){
        double saveHoodAngle=hoodAngle;
        double volt=hoodSensor.getVoltage();
        //If last loop there was no voltage(first loop)
        if (lastVolt==-1){
            //initialize lastvolt
            lastVolt=volt;
        }else{//Otherwise calculate position
            if (volt-lastVolt<=-maxVJump){
                rotations++;
                hoodAngle=(rotations+volt/3.3)*servoDegPerRot;
                lastVolt=volt;
            }else if(volt-lastVolt>=maxVJump){
                rotations--;
                hoodAngle=(rotations+volt/3.3)*servoDegPerRot;
                lastVolt=volt;
            }else{
                hoodAngle=(rotations+volt/3.3)*servoDegPerRot;
                lastVolt=volt;
            }
        }
        return hoodAngle-saveHoodAngle;
    }

    /**
     * Transfer artifact to flywheel(move transfer up)
     */
    public void transferUp(){this.transfer.setPower(this.transferpowers[0]);}
    /**
     * Lower Transfer
     */
    public void transferDown(){
        this.transfer.setPower(this.transferpowers[1]);
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
        if (targetSpeed-tolerance<=flywheelDriveEx.getVelocity() && flywheelDriveEx.getVelocity()<=targetSpeed+tolerance){
            return true;
        }
        else{
            return false;
        }
    }
}
