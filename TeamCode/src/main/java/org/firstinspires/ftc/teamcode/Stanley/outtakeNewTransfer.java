package org.firstinspires.ftc.teamcode.Stanley;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Aaron.aprilTagV3;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

public class outtakeNewTransfer {
    //Team color
    String teamColor;
    //April tag processor
    aprilTagV3 apriltag;
    //Outtake flywheel
    public DcMotorEx flywheelDrive;
    //Outtake Hood Servo
    CRServo hoodServo;
    //TODO: get servo RPM
    //Servo RPM
    double servoRPM=50;
    //Degrees changed for every servo rotation
    double servoDegPerRot =20;
    //transfer positions(up, down)
    public static double[] transferpowers ={1,0};
    //hood angle transitions
    //save ms time for hood
    long savemstime=0;
    //save hood angle for starting hood angle
    double savehoodAngle=0;
    //if hood running
    public boolean runninghood=false;
    //hood angle(in degrees)
    public double hoodAngle=0;
    //transfer servo
    public DcMotor transfer;
    //drivetrain motors
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    //auto aim vars
    //  Drive = Error * Gain
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    //vars
    int targetTagID=-1;
    public outtakeNewTransfer(HardwareMap hardwareMap, DcMotorEx flywheelDrive, String teamColor, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, CRServo hoodServo, DcMotor transfer, boolean useTag){
        this.flywheelDrive=flywheelDrive;
        this.teamColor=teamColor;
        this.leftFront=leftFront;
        this.rightFront=rightFront;
        this.leftBack=leftBack;
        this.rightBack=rightBack;
        this.hoodServo=hoodServo;
        this.transfer=transfer;
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
            this.apriltag.setPipeline(4);
            this.apriltag.init();
        }
    }
    /**
     * Autoaim to april tag.
     * Currently using built-in april tag ID process.
     * Includes drivetrain control+emergency cancel switch.
     * Target tag set by teamColor variable in class, "Red" or "Blue"
     * @return False if canceled or teamColor not found, True if successful
     */
    public boolean autoturn(){
        this.apriltag.scanOnce();
        if (!apriltag.hasValidTarget()){
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            return false;
        }
        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double  headingError    = -apriltag.getYaw();

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        double drive=0;
        double twist   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
        double strafe=0;

        double[] speeds = {
                (drive - strafe - twist), //forward-left motor
                (drive + strafe + twist), //forward-right motor
                (drive + strafe - twist), //back-left motor
                (drive - strafe + twist)  //back-right motor
        };

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        // apply the calculated values to the motors.
        leftFront.setPower(speeds[0]);
        rightFront.setPower(speeds[1]);
        leftBack.setPower(speeds[2]);
        rightBack.setPower(speeds[3]);
        return true;
    }

    /**
     * Find optimal launch angle and velocity to hit a target with specific impact angle constraints.
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
                    if (impactAngle > maxImpactAngle) {
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
     * @param override stop current running action and start new action
     * @return if hood is at position
     */
    public boolean setHood(double degrees, boolean override){
        double rotations=(degrees-this.hoodAngle)/this.servoDegPerRot;
        //time needed to rotate for in ms
        double time=Math.abs(rotations*60*1000/this.servoRPM);
        //If overriding to new position and servo is already running
        if (override && runninghood){
            //update current hood angle
            this.updateHoodAngle(degrees);
            //Initialize variables for new position
            this.savemstime=System.currentTimeMillis();
            this.savehoodAngle=this.hoodAngle;
            if (rotations > 0) {
                this.hoodServo.setPower(1);
            } else if (rotations < 0) {
                this.hoodServo.setPower(-1);
            }
        }
        //Exit condition
        if (System.currentTimeMillis()-this.savemstime>=time){
            //Update hood position
            this.updateHoodAngle(degrees);
            //set runninghood to false(no longer running hood)
            this.runninghood=false;
            //stop hood servo
            this.hoodServo.setPower(0);
            return true;
        }
        if (!runninghood) {
            this.savemstime=System.currentTimeMillis();
            this.savehoodAngle=this.hoodAngle;
            if (rotations > 0) {
                this.hoodServo.setPower(1);
            } else if (rotations < 0) {
                this.hoodServo.setPower(-1);
            }
            this.runninghood=true;
        }
        return false;
    }

    /**
     * Helper function to update current angle of hood
     * @param degrees Target degrees the hood is currently trying to get to
     */
    public void updateHoodAngle(double degrees){
        double rotations=(degrees-this.hoodAngle)/this.servoDegPerRot;
        //Update hood position
        double elapsed = System.currentTimeMillis() - this.savemstime;
        double totalRotation = (elapsed * this.servoRPM) / 60000.0;
        this.hoodAngle = this.savehoodAngle + (rotations > 0 ? 1 : -1) * totalRotation * this.servoDegPerRot;
    }

    /**
     * Transfer artifact to flywheel(spin transfer)
     */
    public void transferStart(){
        this.transfer.setPower(this.transferpowers[0]);
    }
    /**
     * Lower Transfer
     */
    public void transferStop(){
        this.transfer.setPower(this.transferpowers[1]);
    }

    /**
     * Spin flywheel to speed
     * @param targetSpeed Target flywheel speed in encoder ticks/sec.
     * @param tolerance Tolerance in ticks/sec. from target speed to return true.
     * @return If flywheel is up to speed.
     */
    public boolean spin_flywheel(double targetSpeed, int tolerance){
        DcMotorEx flywheelDriveEx=this.flywheelDrive;
        flywheelDriveEx.setVelocity(targetSpeed);
        if (targetSpeed-tolerance<=flywheelDriveEx.getVelocity() && flywheelDriveEx.getVelocity()<=targetSpeed+tolerance){
            return true;
        }
        else{
            return false;
        }
    }
}
