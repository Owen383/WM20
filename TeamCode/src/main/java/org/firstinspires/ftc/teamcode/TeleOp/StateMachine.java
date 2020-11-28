package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

public class StateMachine {
	
	public StateMachine(){
	
	}
	
	public enum DriveState {
		STATE_NORMAL,
		STATE_0,
		STATE_90,
		STATE_180,
		STATE_270
	}
	
	public enum GripperState {
		STATE_GRIP,
		STATE_EJECT,
		STATE_OPEN
	}
	
	public enum ShooterState {
		STATE_OFF,
		STATE_TOP_GOAL,
		STATE_POWER_SHOT
	}
	
	public enum FeederState {
		STATE_IDLE,
		STATE_RESET,
		STATE_FEED,
		STATE_DELAY
	}
	
	public GripperState currentGripperState = GripperState.STATE_OPEN;
	public ShooterState currentShooterState = ShooterState.STATE_OFF;
	public FeederState currentFeederState = FeederState.STATE_RESET;
	public DriveState currentDriveState = DriveState.STATE_NORMAL;
	
	public ElapsedTime driveTime = new ElapsedTime();
	public ElapsedTime gripperTime = new ElapsedTime();
	public ElapsedTime shooterTime = new ElapsedTime();
	public ElapsedTime feederTime = new ElapsedTime();
	
	public void newState(DriveState newState) {
		currentDriveState = newState;
		driveTime.reset();
	}
	
	public void newState(GripperState newState) {
		currentGripperState = newState;
		gripperTime.reset();
	}
	
	public void newState(ShooterState newState) {
		currentShooterState = newState;
		shooterTime.reset();
	}
	
	public void newState(FeederState newState) {
		currentFeederState = newState;
		feederTime.reset();
	}
}
