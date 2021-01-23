package org.firstinspires.ftc.teamcode.Autonomous.AutoUtils;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Autonomous.AutoUtils.Utils;

public class Claw {

    private Servo servo;
    private final static double SERVO_CLAW_HOME = 0.23;
    private final static double SERVO_CLAW_MIN_RANGE = 0.23;
    private final static double SERVO_CLAW_MAX_RANGE = 0.38;
    private double SERVO_CLAW_SPEED = 0.1;

    public Claw(String id){

        try {
            servo = Utils.hardwareMap.get(Servo.class, id);
        }
        catch (Exception e){
            throw new Error("Cannot find servo with id: " + id + "\n. Could not initialize claw.");
        }

    }

    public void openFull(){
        servo.setPosition(SERVO_CLAW_MAX_RANGE);
    }
    public void closeFull(){
        servo.setPosition(SERVO_CLAW_MIN_RANGE);
    }
    public void setSpeed(double speed){ SERVO_CLAW_SPEED = speed; }
}
