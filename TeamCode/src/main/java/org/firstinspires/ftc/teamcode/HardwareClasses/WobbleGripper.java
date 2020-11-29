package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RingBuffer;

public class WobbleGripper {

    public Servo gripper;
    public Servo lifter;
    private double armPosition;
    private static final double GRIPPER_CLOSED = 0.0;
    private static final double GRIPPER_EJECT = .9;
    private static final double GRIPPER_OPEN = 0.5;
    private static final double ARM_UP = .4;
    private static final double ARM_DOWN = 0.8;
    private static final double SERVO_RANGE = .05;
    private RingBuffer<Double> timeRing = new RingBuffer<>(20, 0.0);

    public WobbleGripper(Servo gripper, Servo lifter){
        this.gripper = gripper;
        this.lifter = lifter;
    }

    public void grip(){ gripper.setPosition(GRIPPER_CLOSED); }

    public void eject(){ gripper.setPosition(GRIPPER_EJECT); }

    public boolean isEjected(){
        return (gripper.getPosition() > GRIPPER_EJECT);
    }

    public void open(){ gripper.setPosition(GRIPPER_OPEN);}

    public boolean isOpen(){
        return gripper.getPosition() == GRIPPER_OPEN;
    }
    
    public void stopArm(){
        lifter.setPosition(lifter.getPosition());
    }

    public void armControl(double deltaPosition){
        double currentTime = System.currentTimeMillis();
        double deltaMili = currentTime - timeRing.getValue(currentTime);
        armPosition = lifter.getPosition() + deltaPosition * deltaMili;
        if(armPosition < ARM_UP){ armPosition = ARM_UP; }
        else if (armPosition > ARM_DOWN){ armPosition = ARM_DOWN; }
        lifter.setPosition(armPosition);
    }

    public void armUp() {
        lifter.setPosition(ARM_UP);
    }

    public boolean isArmUp() {
        return ARM_UP - SERVO_RANGE <= lifter.getPosition() && lifter.getPosition() >= ARM_UP + SERVO_RANGE;
    }

    public void armDown() {
        lifter.setPosition(ARM_DOWN);
    }

    public boolean isArmDown() {
        return ARM_DOWN - SERVO_RANGE <= lifter.getPosition() && lifter.getPosition() >= ARM_DOWN + SERVO_RANGE;
    }

}
