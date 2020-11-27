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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.HardwareClasses.MecanumControl;
import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter;
import org.firstinspires.ftc.teamcode.HardwareClasses.WobbleGripper;
import org.firstinspires.ftc.teamcode.teamcodecopy.Gyro;
import org.firstinspires.ftc.utilities.IMU;
import org.firstinspires.ftc.utilities.Utils;

@TeleOp(name = "Full TeleOp", group = "TeleOp")
//@Disabled
public class FullTeleOp extends OpMode {
	
	private final ElapsedTime runtime = new ElapsedTime();
	private ElapsedTime feederTime = new ElapsedTime();
	private IMU imu;
	private Gyro gyro;
	private Controller driver;
	private Controller operator;
	private MecanumControl robot;
	private WobbleGripper wobble;
	private Shooter shooter;
	private DcMotor shooterOne;
	private DcMotor shooterTwo;
	private DcMotor intake;
	private Servo gripper;
	private Servo lifter;
	private Servo feeder;
	private double armPos;
	private GripperState currentGripperState = GripperState.STATE_OPEN;
	private ShooterState currentShooterState = ShooterState.STATE_OFF;
	private FeederState currentFeederState = FeederState.STATE_RESET;
	
	
	@Override
	public void init() {
		telemetry.addData("Status", "Initialized");
		
		gripper = hardwareMap.get(Servo.class, "gripper");
		lifter = hardwareMap.get(Servo.class, "lifter");
		feeder = hardwareMap.get(Servo.class, "feeder");
		
		driver = new Controller(gamepad1);
		operator = new Controller(gamepad2);
		
		DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
		DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontright");
		DcMotor backLeft = hardwareMap.get(DcMotor.class, "backleft");
		DcMotor backRight = hardwareMap.get(DcMotor.class, "backright");
		
		shooterOne = hardwareMap.get(DcMotor.class, "shooter_one");
		shooterTwo = hardwareMap.get(DcMotor.class, "shooter_two");
		intake = hardwareMap.get(DcMotor.class, "intake");
		
		frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
		frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		backRight.setDirection(DcMotorSimple.Direction.FORWARD);
		backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		
		
		// IMU (Inertial Measurement Unit)
		Utils.setHardwareMap(hardwareMap);
		imu = new IMU("imu");
		gyro = new Gyro(imu, 0);
		wobble = new WobbleGripper(gripper, lifter);
		shooter = new Shooter(shooterOne, shooterTwo, feeder);
		robot = new MecanumControl(frontLeft, frontRight, backLeft, backRight, gyro);
	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init_loop() {
	}
	
	/*
	 * This method will be called ONCE when start is pressed
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
	 */
	@Override
	public void start() {
		runtime.reset();
		robot.resetGyro();
	}
	
	/*
	 * This method will be called repeatedly in a loop
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
	 */
	@Override
	public void loop() {
		telemetry.addData("Status", "Run Time: " + runtime.toString());
		
		Controller.Thumbstick driverThumbstickR = driver.getRightThumbstick();
		Controller.Thumbstick driverThumbstickL = driver.getLeftThumbstick();
		Controller.Thumbstick operatorThumbstickR = operator.getRightThumbstick();
		Controller.Thumbstick operatorThumbstickL = operator.getLeftThumbstick();
		
		driverThumbstickR.setShift(gyro.getModAngle());
		driverThumbstickL.setShift(0);
		operatorThumbstickR.setShift(0);
		operatorThumbstickL.setShift(0);
		
		//gripper state machine
		switch (currentGripperState) {
			case STATE_OPEN:
				if (operator.rightBumperPress()) {
					newState(GripperState.STATE_GRIP);
				} else if (operator.leftBumperPress()) {
					newState(GripperState.STATE_EJECT);
				} else {
					wobble.open();
					telemetry.addData("state = ", "open");
				}
				break;
			case STATE_GRIP:
				if (operator.rightBumperPress()) {
					newState(GripperState.STATE_OPEN);
					break;
				}
				if (operator.leftBumperPress()) {
					newState(GripperState.STATE_EJECT);
					break;
				}
				wobble.grip();
				telemetry.addData("state = ", "grip");
				
				break;
			case STATE_EJECT:
				if (wobble.isEjected()) {
					newState(GripperState.STATE_OPEN);
				}
				wobble.eject();
				telemetry.addData("state = ", "EJECT");
				break;
			
		}
		
		//arm control
		wobble.armControl((gamepad2.right_trigger) / -50 - (gamepad2.left_trigger) / -50);
		telemetry.addData("arm position = ", armPos);
		
		//shooter state machine
		switch (currentShooterState) {
			case STATE_OFF:
				if (driver.triangle()) {
					newState(ShooterState.STATE_TOP_GOAL);
					break;
				}
				shooter.setPower(0);
				telemetry.addData("shooter state = ", "off");
				
				break;
			case STATE_TOP_GOAL:
				if (driver.left()) {
					newState(ShooterState.STATE_POWER_SHOT);
					break;
				}
				if (driver.square()) {
					newState(ShooterState.STATE_OFF);
					break;
				}
				shooter.setPower(1.0);
				telemetry.addData("shooter state = ", "power shot");
				break;
			case STATE_POWER_SHOT:
				if (driver.right()) {
					newState(ShooterState.STATE_TOP_GOAL);
					break;
				}
				if (driver.square()) {
					newState(ShooterState.STATE_OFF);
					break;
				}
				shooter.setPower(.75);
				telemetry.addData("shooter state = ", "top goal");
				
				break;
		}
		
		//feeder state machine
        switch (currentFeederState) {
	        case STATE_IDLE:
	        	if(operator.square()) {
			        newState(FeederState.STATE_FEED);
			        break;
		        }
	        	shooter.resetFeeder();
	        	break;
	        case STATE_FEED:
		        if(shooter.feeder.getPosition() < .35){
			        newState(FeederState.STATE_RESET);
			        break;
		        }
		        shooter.feedRing();
		        break;
	        case STATE_RESET:
	        	if(shooter.feeder.getPosition() > .35){
	        		newState(FeederState.STATE_DELAY);
	        		break;
		        }
	        	shooter.resetFeeder();
	        	break;
	        case STATE_DELAY:
	        	if(shooter.isDelayComplete()){
	        		newState(FeederState.STATE_IDLE);
			        break;
		        }
	        	shooter.resetFeeder();
	        	break;
        }

        telemetry.addData("feeder position = ", feeder.getPosition());

		telemetry.addData("rpm = ", Math.abs(shooter.getRPM()));

		if (driver.crossToggle()) {
			intake.setPower(1.0);
		} else {
			intake.setPower(0);
		}
		
		double drive = driverThumbstickR.getInvertedShiftedY();
		double strafe = driverThumbstickR.getInvertedShiftedX();
		double turn = driverThumbstickL.getInvertedShiftedX();
		
		telemetry.addData("drive = ", drive);
		telemetry.addData("strafe = ", strafe);
		telemetry.addData("turn = ", turn);
		
		double precision = (((gamepad1.right_trigger + 1) / -2) + 1.5);
		
		robot.setPowerTele(driverThumbstickR.getInvertedShiftedY(), driverThumbstickR.getInvertedShiftedX(), gamepad1.left_stick_x * -1, precision);
		
		telemetry.update();
	}
	
	private void newState(GripperState newState) {
		currentGripperState = newState;
	}
	
	private void newState(ShooterState newState) {
		currentShooterState = newState;
	}
	
	private void newState(FeederState newState){
	    currentFeederState = newState;
	    shooter.feederDelay.reset();
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
	
	private enum FeederState {
		STATE_IDLE,
		STATE_RESET,
		STATE_FEED,
		STATE_DELAY
	}
}
