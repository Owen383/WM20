package org.firstinspires.ftc.teamcode.HardwareClasses;

import org.firstinspires.ftc.utilities.RingBuffer;
import org.firstinspires.ftc.utilities.IMU;
import org.firstinspires.ftc.utilities.MathUtils;

public class Gyro {

    private IMU imu;
    private double datum;
    private RingBuffer<Double> timeRing = new RingBuffer<>(4, 0.0);
    private RingBuffer<Double> angleRing = new RingBuffer<>(4, 0.0);

    public Gyro(IMU imu, double datum) {
        this.imu = imu;
        this.datum = datum;
    }

    public void setImu(IMU imu) {
        this.imu = imu;
    }

    public void setDatum(double datum) {
        this.datum = datum;
    }
    
    public void reset() { datum = imu.getAngle(); }

    public double getRawAngle() {
        return imu.getAngle() - datum;
    }

    public double getModAngle() {
        return (imu.getAngle() - datum) % 360;
    }
    
    public double rateOfChange(){
        double retVal;

        double currentTime = System.currentTimeMillis();
        double deltaMili = currentTime - timeRing.getValue(currentTime);
        double deltaSeconds = deltaMili / 1000.0;

        double currentAngle = getRawAngle();
        double deltaAngle = currentAngle - angleRing.getValue(currentAngle);

        retVal = deltaAngle / deltaSeconds;

        return retVal;
    }

    //TODO Make this work with more accuracy. Curse you, floorMod(int)!
    public double absToRel(int targetAbsoluteAngle){
        double retval = MathUtils.floorModDouble((360) + imu.getAngle(), 360);
        return (retval <= 180) ? retval : -1 * (360 - retval);
    }
}
