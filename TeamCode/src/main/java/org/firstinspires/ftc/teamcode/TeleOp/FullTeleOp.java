/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClasses.Controller;
import org.firstinspires.ftc.teamcode.HardwareClasses.MecanumControl;
import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter;
import org.firstinspires.ftc.teamcode.HardwareClasses.WobbleGripper;
import org.firstinspires.ftc.teamcode.HardwareClasses.Gyro;
import org.firstinspires.ftc.utilities.IMU;
import org.firstinspires.ftc.utilities.Utils;

@TeleOp(name = "Full TeleOp", group = "TeleOp")
//@Disabled
public class FullTeleOp extends OpMode {
	
	private final ElapsedTime runtime = new ElapsedTime();
	private final ElapsedTime feederTime = new ElapsedTime();
	private final ElapsedTime armTime = new ElapsedTime();
	private Gyro gyro;
	private Controller driver, operator;
	private MecanumControl robot;
	private WobbleGripper wobble;
	private Shooter shooter;
	private DcMotor intake;
	private GripperState currentGripperState = GripperState.STATE_OPEN;
	private ArmState currentArmState = ArmState.STATE_FULL_CONTROL;
	private ShooterState currentShooterState = ShooterState.STATE_OFF;
	private FeederState currentFeederState = FeederState.STATE_IDLE;
	private DriveState currentDriveState = DriveState.STATE_FULL_CONTROL;
	boolean isFeederLocked = true;
	
	
	@Override
	public void init() {
		telemetry.addData("Status", "Initialized");
		
		Servo gripper = hardwareMap.get(Servo.class, "gripper");
		Servo lifter = hardwareMap.get(Servo.class, "lifter");
		Servo feeder = hardwareMap.get(Servo.class, "feeder");
		Servo feederLock = hardwareMap.get(Servo.class, "feeder_lock");
		
		driver = new Controller(gamepad1);
		operator = new Controller(gamepad2);
		
		DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
		DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontright");
		DcMotor backLeft = hardwareMap.get(DcMotor.class, "backleft");
		DcMotor backRight = hardwareMap.get(DcMotor.class, "backright");
		
		DcMotor shooterOne = hardwareMap.get(DcMotor.class, "shooter_one");
		DcMotor shooterTwo = hardwareMap.get(DcMotor.class, "shooter_two");
		intake = hardwareMap.get(DcMotor.class, "intake");
		
		Utils.setHardwareMap(hardwareMap);
		IMU imu = new IMU("imu");
		gyro = new Gyro(imu, 0);
		wobble = new WobbleGripper(gripper, lifter);
		shooter = new Shooter(shooterOne, shooterTwo, feeder, feederLock);
		robot = new MecanumControl(frontLeft, frontRight, backLeft, backRight, gyro);
		wobble.armUp();
	}
	
	@Override
	public void init_loop() {
	}
	
	@Override
	public void start() {
		runtime.reset();
		robot.resetGyro();
	}
	
	@Override
	public void loop() {
		
		Controller.Thumbstick driverRightStick = driver.getRightThumbstick();
		Controller.Thumbstick driverLeftStick = driver.getLeftThumbstick();
		Controller.Thumbstick operatorThumbstickR = operator.getRightThumbstick();
		Controller.Thumbstick operatorThumbstickL = operator.getLeftThumbstick();
		
		driverRightStick.setShift(gyro.getModAngle());
		driverLeftStick.setShift(0);
		operatorThumbstickR.setShift(0);
		operatorThumbstickL.setShift(0);
		
		//gripper state machine
		switch (currentGripperState) {
			case STATE_OPEN:
				if (operator.RBPress()) {
					newState(GripperState.STATE_GRIP);
					break;
				}if (operator.LBPress()) {
					newState(GripperState.STATE_EJECT);
					break;
				}
				wobble.open();
				telemetry.addData("wobble state = ", "open");
				break;
			case STATE_GRIP:
				if (operator.RBPress()) {
					newState(GripperState.STATE_OPEN);
					break;
				}
				if (operator.LBPress()) {
					newState(GripperState.STATE_EJECT);
					break;
				}
				wobble.grip();
				telemetry.addData("wobble state = ", "grip");
				break;
			case STATE_EJECT:
				if (wobble.isEjected()) {
					newState(GripperState.STATE_OPEN);
					break;
				}
				wobble.eject();
				telemetry.addData("wobble state = ", "EJECT");
				break;
		}
		
		
		//arm state machine
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
		
		//shooter state machine
		switch (currentShooterState) {
			case STATE_OFF:
				if (operator.trianglePress()) {
					newState(ShooterState.STATE_TOP_GOAL);
					break;
				}
				shooter.setPower(0);
				telemetry.addData("shooter state = ", "off");
				break;
			case STATE_TOP_GOAL:
				if (operator.left()) {
					newState(ShooterState.STATE_POWER_SHOT);
					break;
				}
				if (operator.trianglePress()) {
					newState(ShooterState.STATE_OFF);
					break;
				}
				shooter.setPower(-1.0);
				telemetry.addData("shooter state = ", "power shot");
				break;
			case STATE_POWER_SHOT:
				if (operator.right()) {
					newState(ShooterState.STATE_TOP_GOAL);
					break;
				}
				if (operator.trianglePress()) {
					newState(ShooterState.STATE_OFF);
					break;
				}
				shooter.setPower(-.75);
				telemetry.addData("shooter state = ", "top goal");
				break;
		}
		
		telemetry.addData("flywheel rpm = ", Math.abs(shooter.getRPM()));
		
		//feeder state machine
		switch (currentFeederState) {
			case STATE_IDLE:
				if (operator.square()) {
					newState(FeederState.STATE_FEED);
					break;
				}
				shooter.resetFeeder();
				if(feederTime.seconds() > .8){
					shooter.lockFeeder();
					isFeederLocked = true;
				}else{
					isFeederLocked = false;
				}
				telemetry.addData("feeder state = ", "idle");
				break;
			case STATE_FEED:
				if (isFeederLocked) {
					if (feederTime.seconds() > .2) {
						newState(FeederState.STATE_RESET);
						break;
					}
					if (feederTime.seconds() > .1) {
						shooter.feedRing();
					}
				}else{
					if (feederTime.seconds() > .1) {
						newState(FeederState.STATE_RESET);
						break;
					}
					shooter.feedRing();
				}
				shooter.unlockFeeder();
				telemetry.addData("feeder state = ", "feed");
				break;
			case STATE_RESET:
				if (feederTime.seconds() > .13) {
					newState(FeederState.STATE_IDLE);
					break;
				}
				shooter.resetFeeder();
				shooter.unlockFeeder();
				telemetry.addData("feeder state = ", "reset");
				break;
		}
		
		telemetry.addData("feeder lock position = ", shooter.feederLock.getPosition());
		
		if (operator.crossToggle()) {
			intake.setPower(1.0);
		} else {
			intake.setPower(0);
		}
		
		
		double precision = (((driver.RT() + 1) / -2) + 1.5);
		
		//drive state machine
		switch (currentDriveState) {
			case STATE_FULL_CONTROL:
				if (driver.up()) {
					newState(DriveState.STATE_0);
				}
				if (driver.left()) {
					newState(DriveState.STATE_90);
				}
				if (driver.down()) {
					newState(DriveState.STATE_180);
				}
				if (driver.right()) {
					newState(DriveState.STATE_270);
				} else {
					robot.setPowerTele(driverRightStick.getInvertedShiftedY(), driverRightStick.getInvertedShiftedX(), driverLeftStick.getInvertedX(), precision);
				}
				telemetry.addData("drive state = ", "full control");
				break;
			case STATE_0:
				if ((driverLeftStick.isInput() || gyro.getModAngle() == 0)) {
					newState(DriveState.STATE_FULL_CONTROL);
				}
				if (driver.left()) {
					newState(DriveState.STATE_90);
				}
				if (driver.down()) {
					newState(DriveState.STATE_180);
				}
				if (driver.right()) {
					newState(DriveState.STATE_270);
				}
				
				robot.setPowerAuto(driverRightStick.getInvertedShiftedY(), driverRightStick.getInvertedShiftedX(), robot.closestTarget(0), precision);
				telemetry.addData("drive state = ", "0º");
				break;
			case STATE_90:
				if ((driverLeftStick.isInput() || gyro.getModAngle() == 0)) {
					newState(DriveState.STATE_FULL_CONTROL);
				}
				if (driver.up()) {
					newState(DriveState.STATE_0);
				}
				if (driver.down()) {
					newState(DriveState.STATE_180);
				}
				if (driver.right()) {
					newState(DriveState.STATE_270);
				}
				
				robot.setPowerAuto(driverRightStick.getInvertedShiftedY(), driverRightStick.getInvertedShiftedX(), robot.closestTarget(90), precision);
				telemetry.addData("drive state = ", "90º");
				break;
			case STATE_180:
				if ((driverLeftStick.isInput() || gyro.getModAngle() == 0)) {
					newState(DriveState.STATE_FULL_CONTROL);
				}
				if (driver.left()) {
					newState(DriveState.STATE_90);
				}
				if (driver.up()) {
					newState(DriveState.STATE_0);
				}
				if (driver.right()) {
					newState(DriveState.STATE_270);
				}
				
				robot.setPowerAuto(driverRightStick.getInvertedShiftedY(), driverRightStick.getInvertedShiftedX(), robot.closestTarget(180), precision);
				telemetry.addData("drive state = ", "180º");
				break;
			case STATE_270:
				if ((driverLeftStick.isInput() || gyro.getModAngle() == 0)) {
					newState(DriveState.STATE_FULL_CONTROL);
				}
				if (driver.left()) {
					newState(DriveState.STATE_90);
				}
				if (driver.down()) {
					newState(DriveState.STATE_180);
				}
				if (driver.up()) {
					newState(DriveState.STATE_0);
				}
				
				robot.setPowerAuto(driverRightStick.getInvertedShiftedY(), driverRightStick.getInvertedShiftedX(), robot.closestTarget(270), precision);
				telemetry.addData("drive state = ", "270º");
				break;
		}
		
		telemetry.update();
	}
	
	private void newState(DriveState newState) {
		currentDriveState = newState;
	}
	
	private void newState(GripperState newState) {
		currentGripperState = newState;
	}
	
	private void newState(ArmState newState) {
		currentArmState = newState;
		armTime.reset();
	}
	
	private void newState(ShooterState newState) {
		currentShooterState = newState;
	}
	
	private void newState(FeederState newState) {
		currentFeederState = newState;
		feederTime.reset();
	}
	
	private enum DriveState {
		STATE_FULL_CONTROL,
		STATE_0,
		STATE_90,
		STATE_180,
		STATE_270
	}
	
	private enum GripperState {
		STATE_GRIP,
		STATE_EJECT,
		STATE_OPEN
	}
	
	private enum ArmState {
		STATE_FULL_CONTROL,
		STATE_DOWN,
		STATE_UP,
		STATE_DELAY
	}
	
	private enum ShooterState {
		STATE_OFF,
		STATE_TOP_GOAL,
		STATE_POWER_SHOT
	}
	
	private enum FeederState {
		STATE_IDLE,
		STATE_RESET,
		STATE_FEED
	}
}
