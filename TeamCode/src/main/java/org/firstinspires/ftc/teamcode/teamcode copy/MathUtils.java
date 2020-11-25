package org.firstinspires.ftc.teamcode;

import static java.lang.Math.floorMod;

public class MathUtils {

    public static double sigmoid(double rawNumber, double range, double outputRange){
        return (outputRange / (1 + Math.pow(range, -rawNumber))) - outputRange / 2;
    }

    public static double degsToRads(Double degs){
        return degs * (Math.PI / 180);
    }

    //TODO Make these actually have input for accuracy like a normal function
    public static double floorModDouble(double dividend, double divisor){
        return floorMod(Math.round(dividend * 1e6), Math.round(divisor * 1e6)) / 1e6;
    }

    public static double floorModDouble(double dividend, int divisor){
        return floorMod(Math.round(dividend * 1e6), divisor) / 1e6;
    }

    public static double floorModDouble(int dividend, double divisor){
        System.out.println("pls work, github");
        return floorMod(dividend, Math.round(divisor * 1e6)) / 1e6;

    }


}
