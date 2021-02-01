package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private DcMotor intakeDrive;
    private Servo reachOne;
    private Servo reachTwo;
    private final static double RETRACTED = 0.29;
    private final static double DEPLOYED = 0.0;
    private final static double INTAKE_ON = 1.0;
    private final static double INTAKE_REVERSE = .75;
    
    private IntakeState currentIntakeState = IntakeState.STATE_OFF;
    private ReachState currentReachState = ReachState.STATE_RETRACT;

    public Intake(DcMotor intakeDrive, Servo reachOne, Servo reachTwo){
        intakeDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        this.intakeDrive = intakeDrive;
        this.reachOne = reachOne;
        this.reachTwo = reachTwo;
    }

    public void retractReach(){
        reachOne.setPosition(RETRACTED);
        reachTwo.setPosition(-RETRACTED+1);
    }

    public void deployReach(){
        reachOne.setPosition(DEPLOYED);
        reachTwo.setPosition(-DEPLOYED+1);
    }
    
    public void reachState(boolean deployToggle){
        switch (currentReachState) {
            
            case STATE_RETRACT:
                if (deployToggle) { newState(ReachState.STATE_DEPLOY); break; }
                retractReach();
                break;
            
            case STATE_DEPLOY:
                if (deployToggle) { newState(ReachState.STATE_RETRACT); newState(IntakeState.STATE_OFF); break; }
                deployReach();
                break;
        }
    }
    

    public void intakeOn(){ intakeDrive.setPower(INTAKE_ON); }

    public void intakeOff(){ intakeDrive.setPower(0.0); }

    public void intakeReverse(){ intakeDrive.setPower(INTAKE_ON * -INTAKE_REVERSE); }
    
    public void intakeState(boolean intakeOn, boolean intakeOff, boolean intakeReverse){
        switch (currentIntakeState) {
            
            case STATE_OFF:
                if (intakeOn) { newState(IntakeState.STATE_ON); newState(ReachState.STATE_DEPLOY); break; }
                if (intakeReverse) { intakeReverse(); newState(ReachState.STATE_DEPLOY); break; }
                intakeOff();
                break;
                
            case STATE_ON:
                if (intakeReverse) { intakeReverse(); break; }
                if (intakeOff) { newState(IntakeState.STATE_OFF); break; }
                intakeOn();
                break;
        }
    }
    
    
    
    
    private void newState(IntakeState newState) {
        currentIntakeState = newState;
    }
    
    private void newState(ReachState newState) {
        currentReachState = newState;
    }
    
    private enum IntakeState {
        STATE_OFF,
        STATE_ON,
        STATE_REVERSE
    }
    
    private enum ReachState {
        STATE_RETRACT,
        STATE_DEPLOY
    }

}
