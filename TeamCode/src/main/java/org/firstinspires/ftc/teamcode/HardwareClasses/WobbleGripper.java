package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.Servo;

public class WobbleGripper {

    private Servo gripper;
    private Servo lifter;
    private double armPosition;
    private static final double GRIPPER_CLOSED = 0.0;
    private static final double GRIPPER_EJECT = 1.0;
    private static final double GRIPPER_OPEN = 0.7;
    private static final double ARM_UP = 1.0;
    private static final double ARM_DOWN = 0.0;
    private static final double SERVO_RANGE = .01;

    public WobbleGripper(Servo gripper, Servo lifter){
        this.gripper = gripper;
        this.lifter = lifter;
    }

    public void grip(){ gripper.setPosition(GRIPPER_CLOSED); }

    public void eject(){ gripper.setPosition(GRIPPER_EJECT); }

    public boolean isEjected(){
        return GRIPPER_EJECT - SERVO_RANGE <= gripper.getPosition() && gripper.getPosition() <= GRIPPER_EJECT + SERVO_RANGE;
    }

    public void open(){ gripper.setPosition(GRIPPER_OPEN);}

    public boolean isOpen(){
        return gripper.getPosition() == GRIPPER_OPEN;
    }

    public void armControl(double deltaPosition){
        armPosition = armPosition + deltaPosition;
        if(armPosition > 1.0){ armPosition = 1.0; }
        else if (armPosition < -1.0){ armPosition = -1.0; }
        lifter.setPosition(armPosition);
    }

    public void armUp() {
        lifter.setPosition(ARM_UP);
    }

    public boolean isArmUp() {
        return ARM_UP - SERVO_RANGE <= lifter.getPosition() && lifter.getPosition() >= ARM_UP + SERVO_RANGE;
    }

    public void setArmDown() {
        lifter.setPosition(ARM_DOWN);
    }

    public boolean isArmDown() {
        return ARM_DOWN - SERVO_RANGE <= lifter.getPosition() && lifter.getPosition() >= ARM_DOWN + SERVO_RANGE;
    }

}
