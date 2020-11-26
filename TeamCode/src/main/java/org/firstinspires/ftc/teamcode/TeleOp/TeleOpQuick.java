package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.HardwareClasses.MecanumControl;
import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter;


@TeleOp(name = "quickly", group="TeleOp")
//@Disabled
public class TeleOpQuick extends OpMode {

    private enum ShooterState{
        STATE_OFF,
        STATE_TOP_GOAL,
        STATE_POWER_SHOT
    }

    private ShooterState currentShooterState = ShooterState.STATE_OFF;

    private DcMotor shooterOne;
    private DcMotor shooterTwo;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
//    private Servo feeder;
    private org.firstinspires.ftc.utilities.IMU imu;

      private Controller driver;
    private Shooter shooter;
    private MecanumControl robot;

    private boolean absoluteControlMode = true;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        shooterOne = hardwareMap.get(DcMotor.class, "shooter_one");
        shooterTwo = hardwareMap.get(DcMotor.class, "shooter_two");
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");
        //imu = new org.firstinspires.ftc.utilities.IMU("imu");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        driver = new Controller(gamepad1);
        shooter = new Shooter(shooterOne, shooterTwo);
        robot = new MecanumControl(frontLeft, frontRight, backLeft, backRight, imu);

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
            shooter.setPower(.4);
        }else{
            shooter.setPower(0.0);
        }

        telemetry.addData("iSq = ", driver.getiSq());
        telemetry.update();

        Controller.Thumbstick rightThumbstick = driver.getRightThumbstick();
        Controller.Thumbstick leftThumbstick = driver.getLeftThumbstick();

        if(absoluteControlMode){
            rightThumbstick.setShift(0);
        }else{
            rightThumbstick.setShift(0);
        }

        double precision =(( (driver.rightTrigger() + 1) / -2) + 1);

        robot.setPower(rightThumbstick.getInvertedShiftedY(), leftThumbstick.getInvertedShiftedX(), rightThumbstick.getInvertedShiftedX(), precision);

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
