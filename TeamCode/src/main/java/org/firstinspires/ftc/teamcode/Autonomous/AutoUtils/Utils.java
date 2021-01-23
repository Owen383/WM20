package org.firstinspires.ftc.teamcode.Autonomous.AutoUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Utils {

    public static HardwareMap hardwareMap;
    public static OpMode opMode;
    public static Telemetry telemetry;
    private static boolean isLinearOpMode;

    // Only use if it is in fact a LinearOpMode
    public static LinearOpMode linearOpMode = null;

    /**
     * Sets the OpMode
     * @param opMode
     */
    public static void setOpMode(OpMode opMode) {
        Utils.opMode = opMode;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;

        isLinearOpMode = (opMode instanceof LinearOpMode);
        if (isLinearOpMode) {
            linearOpMode = (LinearOpMode) opMode;
        }
    }


    public static boolean isActive(){
        if (isLinearOpMode) return linearOpMode.opModeIsActive();
        return true;
    }

    /**
     * I'm lazy
     * @param str
     */
    public static void print(String str){
        System.out.println(str);
    }


   public static double convertInches2Ticks(double ticks){
        return (ticks - 4.38) / 0.0207; // Calculated using desmos
   }

   public static double convertTicks2Inches(double inches){
        return (0.0207 * inches) + 4.38; // Calculated using desmos
   }



    /**
     * @param position
     * @param distance
     * @param acceleration
     * @return
     */
    public static double powerRamp(double position, double distance, double acceleration){
        /**
         * The piece wise function has domain restriction [0, inf] and range restriction [0, 1]
         * Simply returns a proportional constant
         */

        position += 0.01;           // Necessary otherwise we're stuck at position 0 (sqrt(0) = 0)
        double normFactor = 1 / Math.sqrt(0.1 * distance);

        // Modeling a piece wise of power as a function of distance
        double p1 = normFactor * Math.sqrt(acceleration * position);
        double p2 = .8;
        double p3 = normFactor * (Math.cbrt(acceleration * (distance - position)));
        double power = Math.min(Math.min(p1, p2), p3)+0.1;
        //telemetry.addData("Distance", distance);
        //telemetry.addData("Position", position);

        return power;
    }


    /**
     * Super simple method to check toggles on buttons
     * @param current
     * @param previous
     * @return
     */
    public static Boolean buttonTapped(boolean current, boolean previous){
        if (current && !previous )return true;
        else if (!current) return false;
        else return previous;
    }

    public static double map(double x, double a_min, double a_max, double b_min, double b_max){
        return (x - a_min) / (a_max - a_min) * (b_max - b_min) + b_min;
    }



    /**
     * @param baseRGB
     * @param currentRGB
     * @return
     */
    public static double distance2Color(double[] baseRGB, double[] currentRGB){
        return Math.sqrt(Math.pow(baseRGB[0] - currentRGB[0], 2) + Math.pow(baseRGB[1] - currentRGB[1], 2) + Math.pow(baseRGB[2] - currentRGB[2], 2));
    }

    /**
     * @param angle
     * @return coTermAngle
     */
    public static double coTerminal(double angle){
        double coTermAngle = (angle + 180) % 360;
        coTermAngle -= 180;
        return coTermAngle;
    }
}