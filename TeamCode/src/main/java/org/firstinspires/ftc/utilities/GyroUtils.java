package org.firstinspires.ftc.utilities;

public class GyroUtils {

    public static double turnPower(double targetAngle, double currentAngle, double power){
        return MathUtils.sigmoid(targetAngle - currentAngle, 1.09, 2);
    }

}
