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

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClasses.Controller;
import org.firstinspires.ftc.teamcode.HardwareClasses.Gyro;
import org.firstinspires.ftc.teamcode.HardwareClasses.Intake;
import org.firstinspires.ftc.teamcode.HardwareClasses.MecanumChassis;
import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter;
import org.firstinspires.ftc.teamcode.HardwareClasses.WobbleGripper;
import org.firstinspires.ftc.utilities.IMU;
import org.firstinspires.ftc.utilities.Utils;

@TeleOp(name = "OwenKinkyTele", group = "TeleOp")
//@Disabled
public class OwenKinkyTele extends OpMode {
	
	private Controller driver, operator;
	private Controller.Thumbstick driverRightStick, driverLeftStick;
	
	private Gyro gyro;
	private MecanumChassis robot;
	private Shooter shooter;
	private Intake intake;
	private WobbleGripper wobble;
	
	@Override
	public void init() {
		telemetry.addData("Status", "Initialized");
		
		Servo feeder = hardwareMap.get(Servo.class, "feeder");
		Servo feederLock = hardwareMap.get(Servo.class, "feederlock");
		
		Servo reachOne = hardwareMap.get(Servo.class, "outerrollerone");
		Servo reachTwo = hardwareMap.get(Servo.class, "outerrollertwo");
		
		Servo lifter = hardwareMap.get(Servo.class, "lifter");
		Servo gripperOne = hardwareMap.get(Servo.class, "gripperone");
		Servo gripperTwo = hardwareMap.get(Servo.class, "grippertwo");
		
		driver = new Controller(gamepad1);
		operator = new Controller(gamepad2);
		driverRightStick = driver.getRightThumbstick();
		driverLeftStick = driver.getLeftThumbstick();
		driverLeftStick.setShift(0);
		
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
		robot = new MecanumChassis(frontLeft, frontRight, backLeft, backRight, gyro);
		shooter = new Shooter(shooterOne, shooterTwo, feeder, feederLock);
		intake = new Intake(intakeDrive, reachOne, reachTwo);
		wobble = new WobbleGripper(gripperOne, gripperTwo, lifter);
	}
	
	@Override
	public void init_loop() {
		intake.retractReach();
		shooter.lockFeeder();
		shooter.resetFeeder();
		shooter.shooterOff();
		intake.intakeOff();
		robot.setPower(0,0, gamepad1.left_stick_x*-1, .5);
	}
	
	@Override
	public void start() {
		robot.resetGyro();
	}
	
	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public void loop() {
		driverRightStick.setShift(gyro.getModAngle());
		
		boolean intakeOff = operator.trianglePressUpdate() || (operator.leftPressUpdate() || operator.rightPressUpdate());
		boolean shooterOff = operator.crossPressUpdate() || operator.circlePressUpdate();
		
		
		//driver controls
		robot.driveState(driverRightStick.getInvertedShiftedY(), driverRightStick.getInvertedShiftedX(),
				driverLeftStick.getInvertedShiftedX(), driver.RT());
		robot.cardinalState(driver.upPressUpdate(), driver.rightPressUpdate(), driver.downPressUpdate(), driver.leftPressUpdate());
		robot.adjustmentState(driver.RBPressUpdate(), driver.LBPressUpdate(), 10);
		
		
		//operator controls
		shooter.shooterState(operator.trianglePress(), operator.trianglePress() || shooterOff, operator.leftPress(), operator.rightPress());
		shooter.feederState(operator.square());
		
		intake.intakeState(operator.crossPress(), operator.crossPress() || intakeOff, operator.circle());
		intake.reachState(operator.RSPressUpdate());
		
		wobble.armState(operator.LT(), operator.RT(), operator.upPressUpdate(), operator.downPressUpdate(), operator.LSPressUpdate());
		wobble.gripperState(operator.RBPressUpdate(), operator.LBPressUpdate());
	}
	
}