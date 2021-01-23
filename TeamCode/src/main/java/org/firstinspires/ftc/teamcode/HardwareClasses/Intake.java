package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private DcMotor intakeDrive;
    private Servo outerRollerOne;
    private Servo outerRollerTwo;
    private final static double RETRACTED = 0.29;
    private final static double DEPLOYED = 0.0;
    private final static double INTAKE_ON = -1.0;

    public Intake(DcMotor intakeDrive, Servo outerRollerOne, Servo outerRollerTwo){
        this.intakeDrive = intakeDrive;
        this.outerRollerOne = outerRollerOne;
        this.outerRollerTwo = outerRollerTwo;
    }

    public void retractOuterRoller(){
        outerRollerOne.setPosition(RETRACTED);
        outerRollerTwo.setPosition(-RETRACTED+1);
    }
    
    public double outerRollerPosition() {
        return (outerRollerOne.getPosition() - outerRollerTwo.getPosition() + 1)/2.0;
    }

    public void deployOuterRoller(){
        outerRollerOne.setPosition(DEPLOYED);
        outerRollerTwo.setPosition(-DEPLOYED+1);
    }

    public void intakeOn(){ intakeDrive.setPower(INTAKE_ON); }

    public void intakeOff(){ intakeDrive.setPower(0.0); }

    public void intakeReverse(){ intakeDrive.setPower(INTAKE_ON * -0.75); }

}
