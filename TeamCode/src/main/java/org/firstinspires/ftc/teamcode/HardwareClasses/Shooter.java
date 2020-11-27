package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PID;
import org.firstinspires.ftc.teamcode.owen.RingBuffer;

public class Shooter {

    private DcMotor shooterOne;
    private DcMotor shooterTwo;
    public Servo feeder;
    private PID shooterPID = new PID(.005, .0000, .000, 100, false);

    private static final double TICKS_PER_ROTATION = 28;
    private static final double RING_FEED = 0.15;
    private static final double RESET = .35;
    private static final int TOP_GOAL = 4000;
    private static final int POWER_SHOT = 3000;
    private static final double DELAY = .4;

    RingBuffer timeRing = new RingBuffer(20);
    RingBuffer positionRing = new RingBuffer(20);
    
    public ElapsedTime feederDelay = new ElapsedTime();

    public Shooter(DcMotor shooterOne, DcMotor shooterTwo, Servo feeder) {
        this.shooterOne = shooterOne;
        this.shooterTwo = shooterTwo;
        this.feeder = feeder;
    }

    public void feedRing(){
        feeder.setPosition(RING_FEED);
    }

    public boolean isRingFed(){
        return (feeder.getPosition() == RING_FEED);
    }

    public void resetFeeder(){
        feeder.setPosition(RESET);
    }

    public boolean isReset(){
        return (feeder.getPosition() == RESET);
    }
    
    public boolean isDelayComplete(){
        return (feederDelay.seconds() > DELAY);
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