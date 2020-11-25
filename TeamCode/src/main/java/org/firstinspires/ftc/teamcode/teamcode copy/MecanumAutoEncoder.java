package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.utilities.IMU;
import org.firstinspires.ftc.utilities.Utils;

@Autonomous(name="Mecanum Encoder", group="Autonomous Linear Opmode")
//@Disabled
public class MecanumAutoEncoder extends LinearOpMode {

    private IMU imu;
    private Servo servo0;

    public void initialize(){

        servo0 = hardwareMap.get(Servo.class, "servo0");

        // IMU (Inertial Measurement Unit)
        Utils.setHardwareMap(hardwareMap);
        imu = new IMU("imu");
    }


    @Override
    public void runOpMode(){

        initialize();
        waitForStart();

        while (opModeIsActive()){

            if (gamepad1.a) servo0.setPosition(1);
            if (gamepad1.b) servo0.setPosition(0);


            telemetry.addData("Initialized", true);
            telemetry.addData("Servo0", servo0.getPosition());
            telemetry.addData("IMU", imu.getAngle());
        }

    }
}
