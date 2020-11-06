package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMap_Example;

// CHAWKS: Name it something useful!
@TeleOp(name = "I like to move", group = "A")
//@Disabled
public class JerseyGirlHolonomic extends HardwareMap_Example {


    @Override
    public void runOpMode() {

        double rStickX;
        double rStickY;
        double lStickX;
        double targetAngle;
        double mag1; // mag is for magnitude
        double mag2;
        double rotationPower;
        double maxPower;
        double scaleDown;


        /*
            CHAWKS: On Driver Station, telemetry will be display!
                    How is this useful for debugging?
        */
        // Send telemetry message to Driver Station
        telemetry.addData("Status: ", "Hit [Init] to Initialize ze bot");    //
        telemetry.update();

        /*
            CHAWKS: Step 0. Initialize OUR ROBOT
        */
        // MUST HAVE THIS LINE BELOW
        init(hardwareMap);

        // Send telemetry message to "Driver Station" signify robot waiting;
        telemetry.addData("Status: ", "Hit [PLAY] to start!");    //
        telemetry.update();

        /*
            CHAWKS: Step 1. Hit Play to run through the code!
        */
        // MUST HAVE THIS LINE BELOW
        waitForStart();

        /*
            CHAWKS: Remember opModeIsActive?! It's a loop!
        */
        // run until the end of the match (driver presses [STOP])
        // MUST HAVE!
        while (opModeIsActive()) {

            rStickX = gamepad1.right_stick_x;
            rStickY = -gamepad1.right_stick_y;
            lStickX = gamepad1.left_stick_x;

            targetAngle = (Math.atan2(rStickY, rStickX));

            rotationPower = -lStickX;
            mag1 = Math.sqrt(Math.pow(rStickX, 2) + Math.pow(rStickY, 2)) * (Math.sin(targetAngle + Math.PI / 4));
            mag2 = Math.sqrt(Math.pow(rStickX, 2) + Math.pow(rStickY, 2)) * (Math.sin(targetAngle - Math.PI / 4));

            maxPower = Math.max(Math.abs(mag1) + Math.abs(rotationPower), Math.abs(mag2) + Math.abs(rotationPower)) + 0.15;
            scaleDown = 1.0;

            if (maxPower > 1)
                scaleDown = 1.0 / maxPower;

            leftFront.setPower((mag2 - rotationPower) * scaleDown);
            leftBack.setPower((mag1 - rotationPower) * scaleDown);
            rightBack.setPower((mag2 + rotationPower) * scaleDown);
            rightFront.setPower((mag1 + rotationPower) * scaleDown);

        }
    }

}
