package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.owen.RingBuffer;

public class Shooter {

    private DcMotor shooterOne;
    private DcMotor shooterTwo;
//    private Servo feeder;

    private static final double TICKS_PER_ROTATION = 3000;
    private static final double RING_FEED = 0.5;
    private static final double RESET = 0.0;
    private static final int TOP_GOAL = 5000;
    private static final int POWER_SHOT = 4000;
    private static final double GAIN = 0.01;
    private static final double SERVO_RANGE = .01;

    RingBuffer timeRing = new RingBuffer(20);
    RingBuffer positionRing = new RingBuffer(20);

    public Shooter(DcMotor shooterOne, DcMotor shooterTwo, Servo feeder) {
        this.shooterOne = shooterOne;
        this.shooterTwo = shooterTwo;
//        this.feeder = feeder;
    }

    public Shooter(DcMotor shooterOne, DcMotor shooterTwo) {
        this.shooterOne = shooterOne;
        this.shooterTwo = shooterTwo;
    }

    public void feedRing(){
    //    feeder.setPosition(RING_FEED);
    }

    public boolean isRingFed(){
        return true;
                //RING_FEED - SERVO_RANGE <= feeder.getPosition() && feeder.getPosition() <= RING_FEED + SERVO_RANGE;
    }

    public void resetFeeder(){
      //  feeder.setPosition(RESET);
    }

    public boolean isReset(){
        return true;
        //RESET- SERVO_RANGE <= feeder.getPosition() && feeder.getPosition() <= RESET + SERVO_RANGE;
         }

    public void setPower(double power){
        shooterOne.setPower(power);
        shooterTwo.setPower(power);
    }

    public void resetEncoders(){
        shooterOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooterOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public long getPosition(){
        return (shooterOne.getCurrentPosition() + shooterTwo.getCurrentPosition()) /  2;
    }

    public double getRPM(){
        double retVal;

        long currentTime = System.currentTimeMillis();
        long deltaMili = currentTime - timeRing.getValue(currentTime);
        double deltaMinutes = deltaMili / 60000.0;

        long currentPosition = getPosition();
        long deltaTicks = currentPosition - positionRing.getValue(currentPosition);
        double deltaRotations = deltaTicks / TICKS_PER_ROTATION;

        retVal = deltaRotations / deltaMinutes;

        return retVal;
    }

    public void setRPM(int targetRPM){ setPower((getRPM() - targetRPM) * GAIN); }

    public void topGoal(){ setRPM(TOP_GOAL); }

    public void powerShot(){ setRPM(POWER_SHOT); }

    public void off(){ setPower(0.0); }

}