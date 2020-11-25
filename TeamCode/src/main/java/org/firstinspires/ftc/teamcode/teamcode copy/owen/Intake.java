package org.firstinspires.ftc.teamcode.owen;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private DcMotor intakeDrive;
    private Servo tubingDeploy;
    private final static double RETRACTED = 0.0;
    private final static double DEPLOYED = 1.0;
    private final static double INTAKE_ON = 1.0;

    public Intake(DcMotor intakeDrive, Servo tubingDeploy){
        this.intakeDrive = intakeDrive;
        this.tubingDeploy = tubingDeploy;
    }

    public void retractTubing(){ tubingDeploy.setPosition(RETRACTED); }

    public boolean isTubingRetracted(){ return tubingDeploy.getPosition() == RETRACTED; }

    public void deployTubing(){ tubingDeploy.setPosition(DEPLOYED); }

    public boolean isTubingDeployed(){ return tubingDeploy.getPosition() == DEPLOYED; }

    public void intakeOn(){ intakeDrive.setPower(INTAKE_ON); }

    public void intakeOff(){ intakeDrive.setPower(0.0); }

    public void intakeReverse(){ intakeDrive.setPower(-INTAKE_ON); }

}
