package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.HardwareClasses.MecanumControl;
import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter;
import org.firstinspires.ftc.teamcode.teamcodecopy.Gyro;
import org.firstinspires.ftc.utilities.IMU;


@TeleOp(name = "quickly", group="TeleOp")
//@Disabled
public class TeleOpQuick extends OpMode {

    private enum GripperState {
        STATE_GRIP,
        STATE_EJECT,
        STATE_OPEN
    }

    private GripperState currentGripperState = GripperState.STATE_OPEN;

    private Servo gripper;
    private Servo arm;
    private final org.firstinspires.ftc.utilities.IMU imu = new IMU("imu");

    private Controller driver;
    private Shooter shooter;
    private MecanumControl robot;
    private final Gyro gyro = new Gyro(imu, 0);

    private double armPos;

    @Override
    public void init() {


        DcMotor shooterOne = hardwareMap.get(DcMotor.class, "shooter_one");
        DcMotor shooterTwo = hardwareMap.get(DcMotor.class, "shooter_two");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontright");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backleft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backright");

        gripper = hardwareMap.get(Servo.class, "gripper");
        arm = hardwareMap.get(Servo.class, "arm");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        driver = new Controller(gamepad1);
        shooter = new Shooter(shooterOne, shooterTwo);
        robot = new MecanumControl(frontLeft, frontRight, backLeft, backRight, gyro);

        telemetry.addData("Status", "Initialized");

    }


    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if (driver.squareToggle()){
            shooter.setPower(1.0);
        }else{
            shooter.setPower(0.0);
        }

        Controller.Thumbstick rightThumbstick = driver.getRightThumbstick();
        Controller.Thumbstick leftThumbstick = driver.getLeftThumbstick();

        rightThumbstick.setShift(0);
        leftThumbstick.setShift(0);

        switch(currentGripperState){
            case STATE_OPEN:
                if(driver.rightBumperPress()){
                    newState(GripperState.STATE_GRIP);
                }else if(driver.leftBumperPress()) {
                    newState(GripperState.STATE_EJECT);
                }else{
                    gripper.setPosition(0.75);
                    telemetry.addData("state = ", "open");
                }
                break;
            case STATE_GRIP:
                if(driver.rightBumperPress()){
                    newState(GripperState.STATE_OPEN);
                }else if(driver.leftBumperPress()) {
                    newState(GripperState.STATE_EJECT);
                }else{
                    gripper.setPosition(0.25);
                    telemetry.addData("state = ", "grip");
                }
                break;
            case STATE_EJECT:
                if(.95 <= gripper.getPosition()){
                    newState(GripperState.STATE_OPEN);
                }else{
                    gripper.setPosition(0.99);
                    telemetry.addData("state = ", "EJECT");
                }
                break;

        }

        telemetry.update();

        armPos = armPos + (gamepad1.right_trigger)/10000 - (gamepad1.left_trigger)/10000;
        if (armPos >= 1.0){ armPos = 1.0; }
        if (armPos <= 0){ armPos = 0; }
        arm.setPosition(armPos);


        double precision = (( (driver.rightTrigger() + 1) / -2) + 1);

        robot.setPower(rightThumbstick.getInvertedShiftedY(), rightThumbstick.getInvertedShiftedX(), gamepad1.left_stick_x*-1, precision);

    }

    private void newState(GripperState newState){
        currentGripperState = newState;
    }
//    public void loop() {
//
//        if(absoluteControlMode){
//            rightThumbstick.setShift(robot.gyro.getModAngle());
//        }else{
//            rightThumbstick.setShift(0);
//        }
//
//        robot.setPower(rightThumbstick.getInvertedShiftedY(), rightThumbstick.getInvertedShiftedX(),
//                leftThumbstick.getInvertedShiftedX());
//
//        switch(currentShooterState){
//            case STATE_OFF:
//                if(driver.trianglePress()){
//                    newState(ShooterState.STATE_TOP_GOAL);
//                }else{
//                    shooter.setPower(0);
//                }
//                break;
//            case STATE_TOP_GOAL:
//                if(driver.left()){
//                    newState(ShooterState.STATE_POWER_SHOT);
//                }else if(driver.trianglePress()){
//                    newState(ShooterState.STATE_OFF);
//                }else{
//                    shooter.setPower(1.0);
//                }
//                break;
//            case STATE_POWER_SHOT:
//                if(driver.right()){
//                    newState(ShooterState.STATE_TOP_GOAL);
//                }else if(driver.trianglePress()){
//                    newState(ShooterState.STATE_OFF);
//                }else{
//                    shooter.setPower(0.50);
//                }
//                break;
//        }
//
//    }
//    private void newState(ShooterState newState){
//        currentShooterState = newState;
//    }
}
