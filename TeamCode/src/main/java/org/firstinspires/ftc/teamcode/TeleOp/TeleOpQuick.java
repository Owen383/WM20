package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Controller;


@TeleOp(name = "quickly", group="TeleOp")
//@Disabled
public class TeleOpQuick extends OpMode {

    private DcMotor shooterOne;
    private DcMotor shooterTwo;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Controller driver;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        shooterOne = hardwareMap.get(DcMotor.class, "shooter_one");
        shooterTwo = hardwareMap.get(DcMotor.class, "shooter_two");
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);



    }


    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        if (gamepad1.x){
            shooterOne.setPower(0.9);
            shooterTwo.setPower(0.9);
        }else{
            shooterOne.setPower(0.0);
            shooterTwo.setPower(0.0);
        }

    }
}
