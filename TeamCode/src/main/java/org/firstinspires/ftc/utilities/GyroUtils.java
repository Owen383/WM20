package org.firstinspires.ftc.utilities;

import android.os.Build;

import androidx.annotation.RequiresApi;

import static java.lang.Math.abs;
import static java.lang.Math.floorMod;

public class GyroUtils {

    public static double turnPower(double targetAngle, double currentAngle, double power){
        return MathUtils.sigmoid(targetAngle - currentAngle, 1.09, 2);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public static double turnTarget(double targetAngle, double currentAngle){
        double simpleTargetDelta = floorMod(Math.round(((360 - targetAngle) + currentAngle) * 1e6), Math.round(360.000 * 1e6)) / 1e6;
        double alternateTargetDelta = -1 * (360 - simpleTargetDelta);
        return StrictMath.abs(simpleTargetDelta) <= StrictMath.abs(alternateTargetDelta) ? currentAngle - simpleTargetDelta : currentAngle - alternateTargetDelta;
    }
    /*
    public static double turnTarget(double targetAngle, double currentAngle){
        double simpleTargetDelta = MathUtils.floorModDouble((360 - targetAngle) + currentAngle, 360);
        double alternateTargetDelta = -1 * (360 - simpleTargetDelta);
        return abs(simpleTargetDelta) <= abs(alternateTargetDelta) ? currentAngle + simpleTargetDelta : currentAngle + alternateTargetDelta;
    }

     */
    /*
    public static double turnTarget(double targetAngle, double currentAngle){
        double simpleTargetDelta = floorMod(Math.round(((360 - targetAngle) + currentAngle) * 1e6), Math.round(360.000 * 1e6)) / 1e6;
        double alternateTargetDelta = -1 * (360 - simpleTargetDelta);
        return abs(simpleTargetDelta) <= abs(alternateTargetDelta) ? currentAngle + simpleTargetDelta : currentAngle + alternateTargetDelta;
    }

     */

}
