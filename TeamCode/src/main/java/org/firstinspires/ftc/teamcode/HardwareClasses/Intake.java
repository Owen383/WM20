package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private DcMotor intakeDrive;
    private Servo outerRollerOne;
    private Servo outerRollerTwo;
    private final static double RETRACTED = 0.8;
    private final static double DEPLOYED = 1.0;
    private final static double INTAKE_ON = -1.0;

    public Intake(DcMotor intakeDrive, Servo outerRollerOne, Servo outerRollerTwo){
        this.intakeDrive = intakeDrive;
        this.outerRollerOne = outerRollerOne;
        this.outerRollerTwo = outerRollerTwo;
    }

    public void retractOuterRoller(){
        outerRollerOne.setPosition(RETRACTED);
        outerRollerTwo.setPosition(-RETRACTED);
    }

    public boolean isTubingRetracted(){ return outerRollerOne.getPosition() == RETRACTED; }

    public void deployOuterRoller(){
        outerRollerOne.setPosition(DEPLOYED);
        outerRollerTwo.setPosition(-DEPLOYED);
    }

    public boolean isTubingDeployed(){ return outerRollerOne.getPosition() == DEPLOYED; }

    public void intakeOn(){ intakeDrive.setPower(INTAKE_ON); }

    public void intakeOff(){ intakeDrive.setPower(0.0); }

    public void intakeReverse(){ intakeDrive.setPower(-INTAKE_ON*.5); }

}
