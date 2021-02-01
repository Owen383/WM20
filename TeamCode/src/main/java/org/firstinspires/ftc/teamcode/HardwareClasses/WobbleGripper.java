package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.utilities.RingBuffer;

public class WobbleGripper {

    public Servo gripperOne;
    public Servo gripperTwo;
    public Servo lifter;
    
    private RingBuffer<Double> timeRing = new RingBuffer<>(20, 0.0);
    
    private double armPosition;
    private static final double GRIPPER_CLOSED = 0.0;
    private static final double GRIPPER_EJECT = .9;
    private static final double GRIPPER_OPEN = 0.5;
    private static final double ARM_UP = .4;
    private static final double ARM_DOWN = 0.8;
    private static final double ARM_FOLD = 0.0;
    private static final double ARM_CONTROL_RATE = -.00005;
    
    private ArmState currentArmState = ArmState.STATE_CONTROL;

    public WobbleGripper(Servo gripperOne, Servo gripperTwo, Servo lifter){
        this.gripperOne = gripperOne;
        this.gripperTwo = gripperTwo;
        this.lifter = lifter;
    }

    public void grip() { gripperOne.setPosition(GRIPPER_CLOSED); }

    public void eject() { gripperOne.setPosition(GRIPPER_EJECT); }

    public void open() { gripperOne.setPosition(GRIPPER_OPEN); }
    
    
    public void wheelGrip() { gripperOne.setDirection(Servo.Direction.FORWARD); }
    
    public void wheelEject() { gripperOne.setDirection(Servo.Direction.REVERSE); }
    

    public void armControl(double deltaPosition){
        
        double currentTime = System.currentTimeMillis();
        double deltaMili = currentTime - timeRing.getValue(currentTime);
        armPosition = lifter.getPosition() + deltaPosition * deltaMili * ARM_CONTROL_RATE;
        
        armPosition = Range.clip(armPosition, ARM_UP, ARM_DOWN);
        
        lifter.setPosition(armPosition);
    }

    public void armUp() { lifter.setPosition(ARM_UP); }

    public void armDown() { lifter.setPosition(ARM_DOWN); }
    
    public void armFold() { lifter.setPosition(ARM_FOLD); }
    

    public void armState(double armControlUp, double armControlDown, boolean armUp, boolean armDown, boolean armFold){
        switch(currentArmState){
            
            case STATE_CONTROL:
                if(armUp) { newState(ArmState.STATE_UP); break; }
                if(armDown) { newState(ArmState.STATE_DOWN); break; }
                if(armFold) { newState(ArmState.STATE_FOLD); break; }
                armControl(armControlUp - armControlDown);
                break;
                
            case STATE_UP:
                if(armControlUp != 0 || armControlDown != 0) { newState(ArmState.STATE_CONTROL); break; }
                if(armDown) { newState(ArmState.STATE_DOWN); break; }
                if(armFold) { newState(ArmState.STATE_FOLD); break; }
                armUp();
                break;
    
            case STATE_DOWN:
                if(armControlUp != 0 || armControlDown != 0) { newState(ArmState.STATE_CONTROL); break; }
                if(armUp) { newState(ArmState.STATE_UP); break; }
                if(armFold) { newState(ArmState.STATE_FOLD); break; }
                armDown();
                break;
    
            case STATE_FOLD:
                if(armControlUp != 0 || armControlDown != 0) { newState(ArmState.STATE_CONTROL); break; }
                if(armUp) { newState(ArmState.STATE_UP); break; }
                if(armDown) { newState(ArmState.STATE_DOWN); break; }
                armFold();
                break;
        }
    }
    
    private void newState(ArmState newState) { currentArmState = newState; }
    
    private enum ArmState {
        STATE_CONTROL,
        STATE_UP,
        STATE_DOWN,
        STATE_FOLD
    }
}
