package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.utilities.PID;
import org.firstinspires.ftc.utilities.RingBufferOwen;

public class Shooter {

    private DcMotor shooterOne;
    private DcMotor shooterTwo;
    public Servo feeder;
    public Servo feederLock;
    private PID shooterPID = new PID(.005, .0000, .000, 100, false);

    private static final double TICKS_PER_ROTATION = 28;
    private static final double RING_FEED = 0.0;
    private static final double RESET = 0.3;
    private static final double FEEDER_LOCK = .42;
    private static final double FEEDER_UNLOCK = 0.25;
    private static final int TOP_GOAL = 4000;
    private static final int POWER_SHOT = 3000;
    private static final double FEEDER_TIME = .2;

    RingBufferOwen timeRing = new RingBufferOwen(20);
    RingBufferOwen positionRing = new RingBufferOwen(20);
    
    public ElapsedTime feederTime = new ElapsedTime();
    
    public Shooter(DcMotor shooterOne, DcMotor shooterTwo, Servo feeder, Servo feederLock) {
        this.shooterOne = shooterOne;
        this.shooterTwo = shooterTwo;
        this.feeder = feeder;
        this.feederLock = feederLock;
    }
    
    public Shooter(DcMotor shooterOne, DcMotor shooterTwo, Servo feeder) {
        this.shooterOne = shooterOne;
        this.shooterTwo = shooterTwo;
        this.feeder = feeder;
    }

    public void feedRing(){
        feeder.setPosition(RING_FEED);
    }

    public void resetFeeder(){
        feeder.setPosition(RESET);
    }
    
    public void lockFeeder(){
        feederLock.setPosition(FEEDER_LOCK);
    }
    
    public void unlockFeeder(){
        feederLock.setPosition(FEEDER_UNLOCK);
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

    public void setRPM(int targetRPM){
        if (shooterPID.update(targetRPM - getRPM()) > 1){
            setPower(1.0);
        }else if (shooterPID.getResult() < -1){
            setPower(-1.0);
        }else {
            setPower(shooterPID.getResult());
        }
    }

    public void topGoal(){ setRPM(TOP_GOAL); }

    public void powerShot(){ setRPM(POWER_SHOT); }

    public void off(){ setPower(0.0); }

}