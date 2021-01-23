package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.utilities.PID;
import org.firstinspires.ftc.utilities.RingBufferOwen;

public class Shooter {

    private DcMotor shooterOne;
    private DcMotor shooterTwo;
    private Servo feeder;
    private Servo feederLock;
    private PID shooterPID = new PID(.0005, 0, .00023, 50, false);

    private static final double TICKS_PER_ROTATION = 28;
    private static final double RING_FEED = 0.0;
    private static final double RESET = 0.3;
    private static final double FEEDER_LOCK = .46;
    private static final double FEEDER_UNLOCK = 0.23;
    private static final int TOP_GOAL = 3500;
    private static final int POWER_SHOT = 3000;
    private static final double FEEDER_TIME = .2;
    private double shooterPower = 0.0;

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

        shooterOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        retVal = Math.abs(deltaRotations / deltaMinutes);

        return retVal;
    }

    public void setRPM(int targetRPM){
        shooterPower = shooterPower + Math.pow(shooterPID.update( targetRPM - getRPM()),3 );
        
        if(shooterPower > 1.0){ shooterPower = 1.0; }
        else if(shooterPower < 0.0){ shooterPower = 0.0; }
        
        setPower(shooterPower);
    }

    public void topGoal(){ setRPM(TOP_GOAL); }

    public void powerShot(){ setRPM(POWER_SHOT); }

    public void off(){ setPower(0.0); }

}