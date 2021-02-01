package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClasses.Gyro;
import org.firstinspires.ftc.teamcode.HardwareClasses.Intake;
import org.firstinspires.ftc.teamcode.HardwareClasses.MecanumChassis;
import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter;
import org.firstinspires.ftc.utilities.IMU;
import org.firstinspires.ftc.utilities.Utils;

@Autonomous(name = "Owen Kinky Auto", group = "Auto")

public class OwenKinkyAuto extends OpMode {
	
	private final ElapsedTime feederTime = new ElapsedTime();
	private final ElapsedTime mainTime = new ElapsedTime();
	
	private Gyro gyro;
	private MecanumChassis robot;
	private Shooter shooter;
	private Intake intake;
	boolean isFeederLocked = true;
	
	private FeederState currentFeederState = FeederState.STATE_IDLE;
	private MainState currentMainState = MainState.STATE_FORWARD;
	
	
	@Override
	public void init() {
		Servo feeder = hardwareMap.get(Servo.class, "feeder");
		Servo feederLock = hardwareMap.get(Servo.class, "feederlock");
		
		Servo outerRollerOne = hardwareMap.get(Servo.class, "outerrollerone");
		Servo outerRollerTwo = hardwareMap.get(Servo.class, "outerrollertwo");
		
		DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
		DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontright");
		DcMotor backLeft = hardwareMap.get(DcMotor.class, "backleft");
		DcMotor backRight = hardwareMap.get(DcMotor.class, "backright");
		
		DcMotor shooterOne = hardwareMap.get(DcMotor.class, "shooterone");
		DcMotor shooterTwo = hardwareMap.get(DcMotor.class, "shootertwo");
		DcMotor intakeDrive = hardwareMap.get(DcMotor.class, "intake");
		
		Utils.setHardwareMap(hardwareMap);
		IMU imu = new IMU("imu");
		gyro = new Gyro(imu, 0);
		shooter = new Shooter(shooterOne, shooterTwo, feeder, feederLock);
		intake = new Intake(intakeDrive, outerRollerOne,outerRollerTwo);
		robot = new MecanumChassis(frontLeft, frontRight, backLeft, backRight, gyro);
	}
	
	public void init_loop(){
		intake.retractOuterRoller();
		intake.intakeOff();
	}
	
	public void start() {
		mainTime.reset();
		robot.resetGyro();
		robot.resetMotors();
		
	}
	
	@Override
	public void loop() {
		
		switch (currentMainState) {
			case STATE_FORWARD:
				robot.strafe(3500,0,0,1,.5,.5,1);
				
				if(robot.currentTicks>1000) {
					intake.deployOuterRoller();
					intake.intakeOn();
				}
				if(robot.isStrafeFinished){
					newState(MainState.STATE_SHOOT);
				}
				break;
			case STATE_SHOOT:
				robot.setPower(0,0,0,0);
				intake.retractOuterRoller();
				intake.intakeOff();
				shooter.topGoal();
				if(shooter.updateRPM() > 3200 && shooter.updateRPM() < 3400){
				
				}
		}
		
		telemetry.addData("flywheel rpm = ", Math.abs(shooter.updateRPM()));
		
		//feeder state machine
		switch (currentFeederState) {
			case STATE_IDLE:
				if(feederTime.seconds() > .8){
					shooter.lockFeeder();
					isFeederLocked = true;
				}else{
					isFeederLocked = false;
					shooter.unlockFeeder();
				}
				shooter.resetFeeder();
				telemetry.addData("feeder state = ", "idle");
				break;
			case STATE_FEED:
				if (isFeederLocked) {
					if (feederTime.seconds() > .3) {
						newState(FeederState.STATE_RESET);
						break;
					}
					if (feederTime.seconds() > 0.1) {
						shooter.feedRing();
					}
				}else{
					if (feederTime.seconds() > .2) {
						newState(FeederState.STATE_RESET);
						break;
					}
					shooter.feedRing();
				}
				shooter.unlockFeeder();
				telemetry.addData("feeder state = ", "feed");
				break;
			case STATE_RESET:
				if (feederTime.seconds() > .15) {
					newState(FeederState.STATE_IDLE);
					break;
				}
				shooter.resetFeeder();
				shooter.unlockFeeder();
				telemetry.addData("feeder state = ", "reset");
				break;
		}
	}
	

	
	private void newState(FeederState newState) {
		currentFeederState = newState;
		feederTime.reset();
	}
	
	private void newState(MainState newState) {
		currentMainState = newState;
		mainTime.reset();
	}
	
	private enum MainState {
		STATE_FORWARD,
		STATE_SHOOT
	}
	
	private enum FeederState {
		STATE_IDLE,
		STATE_RESET,
		STATE_FEED
	}
	
	
	
	
}
