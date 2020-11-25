package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.utilities.IMU;
import org.firstinspires.ftc.utilities.Utils;

import java.lang.Math;

/**
 * First TeleOp
 */
@TeleOp(name = "Basic OpMode", group="TeleOp Iterative Opmode")
//@Disabled
public class MecanumTeleOp extends OpMode {


    private IMU imu;
    private double initAngle;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor shooterOne;
    private DcMotor shooterTwo;
    private Servo goalLock;
    private Servo goalLift;
    private double deadZone = 0.2;
    private boolean useDeadZones = true;
    private boolean bLastCycle = false;
    private double imuDatum;
    private boolean aLastCycle = false;
    private double gain = .00456;
    private Gyro gyro = new Gyro(imu, 0);
    private boolean absoluteControlMode = true;
    private double gyroSetPoint = 0;
    private Telemetry dashboardTelemetry;
    private PID rotationPID = new PID(.017, .00022, .0022, 100, true);



    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        Utils.setHardwareMap(hardwareMap);
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");
        shooterOne = hardwareMap.get(DcMotor.class, "shooterone");
        shooterTwo = hardwareMap.get(DcMotor.class, "shootertwo");

       // goalLift = hardwareMap.get(Servo.class, "goallift");
        goalLock = hardwareMap.get(Servo.class, "goallock");



        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);



          imu = new IMU("imu");
        initAngle = imu.getAngle();

        imuDatum = imu.getAngle();
        gyroSetPoint = imuDatum;

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

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

    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {

        System.out.println("github pls work");

        Controller controller = new Controller(gamepad1);
        MecanumControl mecanumControl = new MecanumControl(frontLeft, frontRight, backLeft, backRight, imu);
        Gyro gyro = new Gyro(imu, imuDatum);

        Controller.Thumbstick rightThumbstick = controller.getRightThumbstick();
        Controller.Thumbstick leftThumbstick = controller.getLeftThumbstick();

        if(absoluteControlMode){
            rightThumbstick.setShift(gyro.getModAngle());
        }else{
            rightThumbstick.setShift(0);
        }

        if(gamepad1.a){
            if(!aLastCycle){
                System.out.println(goalLock.getPosition());
                goalLock.setDirection(Servo.Direction.FORWARD);
                goalLock.setPosition(goalLock.getPosition() + gamepad1.right_stick_x);

                aLastCycle = true;
            }
        }else{
            aLastCycle = false;
        }

        /*
        if(gamepad1.a){
            if(!aLastCycle){
                gyroSetPoint = (gyroSetPoint + 90) % 360;
                aLastCycle = true;
            }
        }else{
            aLastCycle = false;
        }

         */
        /*
        mecanumControl.setStrafe(rightThumbstick.getInvertedShiftedX());
        mecanumControl.setDrive(rightThumbstick.getInvertedShiftedY());
        mecanumControl.setTurn((rotationPID.update(gyro.getRawAngle() - gyroSetPoint) * -1));

        telemetry.addData("Strafe1", rightThumbstick.getInvertedShiftedX());
        telemetry.addData("Drive1", rightThumbstick.getInvertedShiftedY());
        telemetry.addData("Turn1", leftThumbstick.getInvertedX());
        telemetry.addData("IMU", imu.getAngle());
        dashboardTelemetry.addData("Gyro", gyro.getModAngle());
        dashboardTelemetry.addData("PID", rotationPID.getResult());
        dashboardTelemetry.addData("SetPoint", gyroSetPoint);



            shooterOne.setPower(-gamepad1.right_trigger);
            shooterTwo.setPower(-gamepad1.right_trigger);



         */
        telemetry.update();
        dashboardTelemetry.update();
        //mecanumControl.setPower();
    }
}
