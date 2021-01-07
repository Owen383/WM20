package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClasses.Controller;
import org.firstinspires.ftc.teamcode.HardwareClasses.MecanumControl;
import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter;
import org.firstinspires.ftc.teamcode.HardwareClasses.WobbleGripper;
import org.firstinspires.ftc.teamcode.HardwareClasses.Gyro;
import org.firstinspires.ftc.utilities.Utils;

@Autonomous(name = "State Machine Test", group = "Auto")

public class StateMachineAuto extends OpMode {
	
	public ElapsedTime Runtime = new ElapsedTime();
	MecanumControl robot;
	Shooter shooter;
	WobbleGripper wobble;
	Controller operator;
	Controller driver;
	private DriveState currentState = DriveState.STATE_INITIAL;
	private SwitchState currentSwitchState = SwitchState.STATE_INITIAL;
	private FeederState currentFeederState = FeederState.STATE_IDLE;
	private GripperState currentGripperState = GripperState.STATE_OPEN;
	private ShooterState currentShooterState = ShooterState.STATE_OFF;
	private ArmState currentArmState = ArmState.STATE_FULL_CONTROL;
	private final ElapsedTime stateTime = new ElapsedTime();
	private final ElapsedTime switchTime = new ElapsedTime();
	private final ElapsedTime feederTime = new ElapsedTime();
	private final ElapsedTime gripperTime = new ElapsedTime();
	private final ElapsedTime shooterTime = new ElapsedTime();
	private final ElapsedTime armTime = new ElapsedTime();
	private DcMotor frontLeft;
	private DcMotor frontRight;
	private DcMotor backLeft;
	private DcMotor backRight;
	private DcMotor shooterOne;
	private DcMotor shooterTwo;
	private Servo feeder;
	private Servo gripper;
	private Servo lifter;
	private org.firstinspires.ftc.utilities.IMU imu;
	private Gyro gyro;
	
	@Override
	public void init() {
		Utils.setHardwareMap(hardwareMap);
		frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
		frontRight = hardwareMap.get(DcMotor.class, "frontright");
		backLeft = hardwareMap.get(DcMotor.class, "backleft");
		backRight = hardwareMap.get(DcMotor.class, "backright");
		shooterOne = hardwareMap.get(DcMotor.class, "shooter_one");
		shooterTwo = hardwareMap.get(DcMotor.class, "shooter_two");
		
		feeder = hardwareMap.get(Servo.class, "feeder");
		gripper = hardwareMap.get(Servo.class, "gripper");
		lifter = hardwareMap.get(Servo.class, "lifter");
		
		imu = new org.firstinspires.ftc.utilities.IMU("imu");
		gyro = new Gyro(imu, 0);
		
		driver = new Controller(gamepad1);
		operator = new Controller(gamepad2);
		
		robot = new MecanumControl(frontLeft, frontRight, backLeft, backRight, gyro);
		
		shooter = new Shooter(shooterOne, shooterTwo, feeder);
		wobble = new WobbleGripper(gripper, lifter);
		
		telemetry.addData("Status", "Initialized");
		
		
	}

	public void init_loop() {
		
	    switch (currentGripperState){
            case STATE_OPEN:
                if (operator.RBPress()) {
                    newState(GripperState.STATE_GRIP);
                    break;
                }
                wobble.open();
                break;
            case STATE_GRIP:
                if (operator.RBPress()) {
                    newState(GripperState.STATE_OPEN);
                    break;
                }
                wobble.grip();
                break;
        }
        
        switch(currentArmState) {
            case STATE_FULL_CONTROL:
                if(operator.down()){
                    newState(ArmState.STATE_DOWN);
                    break;
                }
                if(operator.up()){
                    newState(ArmState.STATE_UP);
                    break;
                }
                wobble.armControl((operator.LT()) / -20000 - (operator.RT()) / -20000);
                break;
            case STATE_DOWN:
                if(operator.LT() > 0 || operator.RT() > 0){
                    newState(ArmState.STATE_FULL_CONTROL);
                    break;
                }
                if(operator.up()){
                    newState(ArmState.STATE_UP);
                    break;
                }
                wobble.armDown();
                break;
            case STATE_UP:
                if(operator.LT() > 0 || operator.RT() > 0){
                    newState(ArmState.STATE_FULL_CONTROL);
                    break;
                }
                if(operator.down()){
                    newState(ArmState.STATE_DOWN);
                    break;
                }
                
                wobble.armUp();
                break;
            case STATE_DELAY:
                if(armTime.seconds()>.6){
                    newState(ArmState.STATE_FULL_CONTROL);
                    break;
                }
                wobble.stopArm();
        }
		shooter.off();
	}
	
	public void start() {
		Runtime.reset();
		stateTime.reset();
		feederTime.reset();
		gripperTime.reset();
		shooterTime.reset();
		currentGripperState = GripperState.STATE_GRIP;
        currentArmState = ArmState.STATE_UP;
	}

	@Override
	public void loop() {
		
		switch (currentState) {
			case STATE_INITIAL:
				
				newState(DriveState.STATE_RING_STACK);
                telemetry.addData("master state = ", "initial");
				robot.resetMotors();
				break;
			
			case STATE_RING_STACK:
				if (robot.adjustedTicks()>5000) {
					newState(DriveState.STATE_GRAB_GOAL);
					break;
				}
				
				telemetry.addData("master state = ", "ringstack");
				break;
            case STATE_VISION:
                robot.setPowerAuto(0,0,0,0);
                telemetry.addData("master state = ", "vision");
			
		}
		
//		//Wobble Goal Gripper State Machine
//		switch (currentGripperState) {
//			case STATE_OPEN:
//				wobble.open();
//				break;
//			case STATE_GRIP:
//				wobble.grip();
//				break;
//			case STATE_EJECT:
//				if (wobble.isEjected()) {
//					newState(GripperState.STATE_OPEN);
//				} else {
//					wobble.eject();
//				}
//				break;
//		}
		
//		//Shooter State Machine
//		switch (currentShooterState) {
//			case STATE_OFF:
//				shooter.setPower(0);
//				break;
//			case STATE_TOP_GOAL:
//				shooter.topGoal();
//				break;
//			case STATE_POWER_SHOT:
//				shooter.powerShot();
//				break;
//		}
        telemetry.update();
		
	}

	private void newState(DriveState newState) {
		if (currentState != newState) {
			currentState = newState;
			stateTime.reset();
		}
	}

	private void newState(SwitchState newState) {
		if (currentSwitchState != newState) {
			currentSwitchState = newState;
			switchTime.reset();
		}
	}
	
	private void newState(FeederState newState) {
		if (currentFeederState != newState) {
			currentFeederState = newState;
			feederTime.reset();
		}
	}
	
	private void newState(GripperState newState) {
		if (currentGripperState != newState) {
			currentGripperState = newState;
			gripperTime.reset();
		}
	}
	
	private void newState(ShooterState newState) {
		if (currentShooterState != newState) {
			currentShooterState = newState;
			shooterTime.reset();
		}
	}
	
	private void newState(ArmState newState) {
		if (currentArmState != newState) {
			currentArmState = newState;
			armTime.reset();
		}
	}
	
	private enum DriveState {
		STATE_INITIAL,
		STATE_RING_STACK,
		STATE_GRAB_GOAL,
		STATE_VISION,
		STATE_A,
		STATE_B,
		STATE_C
	}
	
	private enum SwitchState {
		STATE_INITIAL,
		STATE_DELIVER_FIRST,
		STATE_EJECT_FIRST_GOAL,
		STATE_DELIVER_SECOND
	}
	
	private enum FeederState {
		STATE_IDLE,
		STATE_FEED,
		STATE_RESET,
		STATE_DELAY
	}
	
	private enum GripperState {
		STATE_GRIP,
		STATE_EJECT,
		STATE_OPEN
	}
	
	private enum ShooterState {
		STATE_OFF,
		STATE_TOP_GOAL,
		STATE_POWER_SHOT
	}
	
	private enum ArmState {
		STATE_FULL_CONTROL,
        STATE_DELAY,
	    STATE_UP,
		STATE_DOWN
	}
	
}
