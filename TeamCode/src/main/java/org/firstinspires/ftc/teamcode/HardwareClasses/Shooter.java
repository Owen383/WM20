package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.utilities.PID;
import org.firstinspires.ftc.utilities.RingBufferOwen;

public class Shooter {

    private DcMotor shooterOne;
    private DcMotor shooterTwo;
    private Servo feeder;
    private Servo feederLock;
    private PID shooterPID = new PID(.0006, 0, .00026, 50, false);

    private static final double TICKS_PER_ROTATION = 28;
    private static final double RING_FEED = 0.05;
    private static final double RESET = 0.38;
    private static final double FEEDER_LOCK = .46;
    private static final double FEEDER_UNLOCK = 0.2;
    private static final int TOP_GOAL = 3300;
    private static final int POWER_SHOT = 3000;
    private static final double FEED_TIME = .2;
    private static final double RESET_TIME = .2;
        private static final double UNLOCK_TIME = .1;
    private double shooterPower = 0.0;
    private boolean isFeederLocked;
    private double shooterRPM;

    RingBufferOwen timeRing = new RingBufferOwen(20);
    RingBufferOwen positionRing = new RingBufferOwen(20);
    private FeederState currentFeederState = FeederState.STATE_IDLE;
    public ShooterState currentShooterState = ShooterState.STATE_OFF;
    
    public ElapsedTime feederTime = new ElapsedTime();
    
    public Shooter(DcMotor shooterOne, DcMotor shooterTwo, Servo feeder, Servo feederLock) {
        this.shooterOne = shooterOne;
        this.shooterTwo = shooterTwo;
        this.feeder = feeder;
        this.feederLock = feederLock;
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
    
    public boolean isFeederLocked() { return feeder.getPosition() == FEEDER_LOCK; }
    
    public void unlockFeeder(){
        feederLock.setPosition(FEEDER_UNLOCK);
    }
    

    public void setPower(double power){
        shooterOne.setPower(power);
        shooterTwo.setPower(power);
    }
    
    public double getPower(){
        return (shooterOne.getPower() + shooterTwo.getPower()) /2;
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

    public double updateRPM(){

        long currentTime = System.currentTimeMillis();
        long deltaMili = currentTime - timeRing.getValue(currentTime);
        double deltaMinutes = deltaMili / 60000.0;

        long currentPosition = getPosition();
        long deltaTicks = currentPosition - positionRing.getValue(currentPosition);
        double deltaRotations = deltaTicks / TICKS_PER_ROTATION;

        shooterRPM = Math.abs(deltaRotations / deltaMinutes);

        return shooterRPM;
    }
    
    public double getRPM(){
        return shooterRPM;
    }

    public void setRPM(int targetRPM){
        shooterPower = shooterPower + Math.pow(shooterPID.update( targetRPM - updateRPM()),3 );
    
        shooterPower = Range.clip(shooterPower,0.0, 1.0);
        setPower(shooterPower);
    }
    

    public void topGoal(){ setRPM(TOP_GOAL); }

    public void powerShot(){ setRPM(POWER_SHOT); }

    public void shooterOff(){ setPower(0.0); }
    
    public void shooterState(boolean shooterOn, boolean shooterOff, boolean powerShot, boolean topGoal){
        switch (currentShooterState) {
            
            case STATE_OFF:
                if (shooterOn || topGoal) { newState(ShooterState.STATE_TOP_GOAL); break; }
                if (powerShot) { newState(ShooterState.STATE_POWER_SHOT); break; }
                shooterOff();
                break;
                
            case STATE_TOP_GOAL:
                if (powerShot) { newState(ShooterState.STATE_POWER_SHOT); break; }
                if (shooterOff) { newState(ShooterState.STATE_OFF); break; }
                topGoal();
                break;
                
            case STATE_POWER_SHOT:
                if (topGoal) { newState(ShooterState.STATE_TOP_GOAL); break; }
                if (shooterOff) { newState(ShooterState.STATE_OFF); break; }
                powerShot();
                break;
        }
    }
    
    
    public void feederState(boolean trigger){
        switch (currentFeederState) {
            
            case STATE_IDLE:
                if(trigger && currentShooterState != ShooterState.STATE_OFF){ newState(FeederState.STATE_FEED); }
                if(feederTime.seconds() > .8){ lockFeeder(); isFeederLocked = false; }
                else{ unlockFeeder(); isFeederLocked = false; }
                resetFeeder();
                break;
                
            case STATE_FEED:
                if (isFeederLocked) {
                    if (feederTime.seconds() > .3) { newState(FeederState.STATE_RESET); }
                    if (feederTime.seconds() > 0.1) { feedRing(); }
                }else{
                    if (feederTime.seconds() > .2) { newState(FeederState.STATE_RESET); }
                    feedRing();
                }
                unlockFeeder();
                break;
                
            case STATE_RESET:
                if (feederTime.seconds() > .15) { newState(FeederState.STATE_IDLE); break; }
                resetFeeder();
                unlockFeeder();
                break;
        }
    }
    
    private void newState(FeederState newState) {
        currentFeederState = newState;
        feederTime.reset();
    }
    
    private void newState(ShooterState newState) {
        currentShooterState = newState;
    }
    
    private enum FeederState {
        STATE_IDLE,
        STATE_RESET,
        STATE_FEED
    }
    
    private enum ShooterState {
        STATE_OFF,
        STATE_TOP_GOAL,
        STATE_POWER_SHOT
    }
    

}