package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.utilities.IMU;

import static java.lang.Math.floorMod;

public class Gyro {

    private IMU imu;
    private double datum;

    public Gyro(IMU imu, double datum) {
        this.imu = imu;
        this.datum = datum;
    }

    public void setImu(IMU imu) {
        this.imu = imu;
    }

    public double getRawAngle() {
        return imu.getAngle() - datum;
    }

    public double getModAngle() {
        return (imu.getAngle() - datum) % 360;
    }

    //TODO Make this work with more accuracy. Curse you, floorMod(int)!
    public double absToRel(int targetAbsoluteAngle){
        double retval = MathUtils.floorModDouble((360) + imu.getAngle(), 360);
        return (retval <= 180) ? retval : -1 * (360 - retval);
    }
}
