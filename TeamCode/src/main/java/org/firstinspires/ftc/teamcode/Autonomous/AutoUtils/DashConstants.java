package org.firstinspires.ftc.teamcode.Autonomous.AutoUtils;
import com.acmerobotics.dashboard.config.Config;

@Config
public class DashConstants {

    public static double p = .015;
    public static double i = .0;
    public static double d = .0004;


    /* ------- VISION --------- */
    // Top rectangle starting percentages
    public static double rectTopX1Percent = 0.75; public static double rectTopX2Percent = 0.9;
    public static double rectTopY1Percent = 0.3; public static double rectTopY2Percent = .38;

    // Bottom rectangle starting percentages
    public static double rectBottomX1Percent = 0.75; public static double rectBottomX2Percent = 0.9;
    public static double rectBottomY1Percent = 0.38; public static double rectBottomY2Percent = 0.42;

    // Generally, orange should be around 90-100
    public static double orangeMax = 110;
    public static double orangeMin = 80;

    public static double power;

    public static double diagnosticInches = 28;
    public static double learning_rate = 0.01;

    public static double diagnostic_ring_count = 0;


}
