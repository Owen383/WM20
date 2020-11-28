package org.firstinspires.ftc.teamcode.teamcodecopy;

public class GyroUtils {

    public static double turnPower(double targetAngle, double currentAngle, double power){
        return org.firstinspires.ftc.teamcode.MathUtils.sigmoid(targetAngle - currentAngle, 1.09, 2);
    }

}
