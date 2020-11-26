package org.firstinspires.ftc.teamcode;

public class GyroUtils {

    public static double turnPower(double targetAngle, double currentAngle, double power){
        return MathUtils.sigmoid(targetAngle - currentAngle, 1.09, 2);
    }

}
