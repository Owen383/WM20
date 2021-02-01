package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareClasses.Controller;
import org.firstinspires.ftc.utilities.IMU;
import org.firstinspires.ftc.utilities.Utils;

@Autonomous(name="Mecanum Encoder", group="Autonomous Linear Opmode")
@Disabled
public class MecanumAutoEncoder extends LinearOpMode {

    private IMU imu;
    private Servo gripper;
    private Servo arm;
    private Controller controller;

    private double armPos;

    private enum GripperState {
        STATE_GRIP,
        STATE_EJECT,
        STATE_OPEN
    }

    private GripperState currentGripperState = GripperState.STATE_OPEN;

    public void initialize(){

        gripper = hardwareMap.get(Servo.class, "gripper");
        arm = hardwareMap.get(Servo.class, "arm");
        controller = new Controller(gamepad1);

        // IMU (Inertial Measurement Unit)
        Utils.setHardwareMap(hardwareMap);
        imu = new IMU("imu");
    }


    @Override
    public void runOpMode(){

        initialize();
        waitForStart();

        while (opModeIsActive()){

            switch(currentGripperState){
                case STATE_OPEN:
                    if(controller.RBPressUpdate()){
                        newState(GripperState.STATE_GRIP);
                    }else if(controller.LBPressUpdate()) {
                        newState(GripperState.STATE_EJECT);
                    }else{
                        gripper.setPosition(0.75);
                        telemetry.addData("state = ", "open");
                    }
                    break;
                case STATE_GRIP:
                    if(controller.RBPressUpdate()){
                        newState(GripperState.STATE_OPEN);
                    }else if(controller.LBPressUpdate()) {
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



            armPos = armPos + (gamepad1.right_trigger)/1000 - (gamepad1.left_trigger)/1000;
            if (armPos >= .75){ armPos = .75; }
            if (armPos <= .35){ armPos = .35; }
            arm.setPosition(armPos);
            telemetry.addData("arm position = ", armPos);
            telemetry.update();
        }


    }
    private void newState(GripperState newState){
        currentGripperState = newState;
    }
}
