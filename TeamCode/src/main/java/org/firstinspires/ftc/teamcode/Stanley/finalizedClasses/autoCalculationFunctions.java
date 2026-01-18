package org.firstinspires.ftc.teamcode.Stanley.finalizedClasses;

public class autoCalculationFunctions {
    /**
     * interpolate flywheel and hood values
     * @param input [distance(ft), lower bound value, upper bound value, lower hood angle, upper hood angle, lower flywheel speed, upper flywheel speed]
     * @return [hood angle, flywheel speed]
     */
    public static double[] interpolate(double[] input){
        double distance=input[0]-input[1];
        double hoodAngle=(input[4]-input[3])*distance/(input[2]-input[1])+input[3];
        double flywheelSpeed=(input[6]-input[5])*distance/(input[2]-input[1])+input[5];
        return new double[]{hoodAngle,flywheelSpeed};
    }
}
