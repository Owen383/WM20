package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.PID;
import org.firstinspires.ftc.teamcode.teamcodecopy.Gyro;

public class MecanumControl {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private PID rotationPID = new PID(.03, 0.0000, .000, 20);

    private Gyro gyro;

    private double drive;
    private double strafe;
    private double turn;
    private double power;
    private double targetAngle;
    private int i = 0;
    double retVal = 0;
    double previousTarget = 0;

    private final static double ACCEL_RATE = 0.001;

    public MecanumControl(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, Gyro gyro) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.gyro = gyro;
    }

    public void resetGyro(){
        gyro.setDatum(gyro.getRawAngle());
        targetAngle = 0;

    }
    
    public double closestTarget(double targetAngle){
        double currentAngle = gyro.getRawAngle();
        double modCurrentAngle = currentAngle % 360;
        double modTargetAngle = targetAngle % 360;
        double adjTargetAngle;
        if(previousTarget != targetAngle) { i=0; }
        if(Math.abs(modTargetAngle - currentAngle) > 180){
            adjTargetAngle = modTargetAngle - 360;
        }else{
            adjTargetAngle = modTargetAngle;
        }
        if(i==0){
            retVal = currentAngle - modCurrentAngle + adjTargetAngle;
        }
        i++;
        previousTarget = targetAngle;

        return retVal;
    }

    public void setPowerTele(double drive, double strafe, double turn, double power){
        this.drive = drive * power;
        this.strafe = strafe * power;
        if (turn > 0) {
            targetAngle = gyro.getRawAngle() + .1 * gyro.rateOfChange();
            this.turn = turn * power;
        }else if (turn < 0){
            targetAngle = gyro.getRawAngle() + .1 * gyro.rateOfChange();
            this.turn = turn * power;
        }else {
            this.turn = rotationPID.update(targetAngle - gyro.getRawAngle()) * Math.abs(power);
        }
        this.power = power;

        double flPower = this.drive - this.strafe - this.turn;
        double frPower = this.drive + this.strafe + this.turn;
        double blPower = this.drive + this.strafe - this.turn;
        double brPower = this.drive - this.strafe + this.turn;
        double maxPower = Math.abs(Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.max(Math.abs(blPower), Math.abs(brPower))));

        if (maxPower > 1) {
            flPower = flPower / maxPower;
            frPower = frPower / maxPower;
            blPower = blPower / maxPower;
            brPower = brPower / maxPower;
        }
        this.frontLeft.setPower(flPower);
        this.frontRight.setPower(frPower);
        this.backLeft.setPower(blPower);
        this.backRight.setPower(brPower);
    }

    public void setPowerTele(double drive, double strafe, double turn){
        setPowerTele(drive, strafe, turn,1.0);
    }
    
    public void setPowerAuto(double drive, double strafe, double targetAngle, double power){
        this.drive = drive * power;
        this.strafe = strafe * power;
        this.targetAngle = targetAngle;
        turn = rotationPID.update(targetAngle - gyro.getRawAngle()) * Math.abs(power);
        this.power = power;
        
        double flPower = this.drive - this.strafe - turn;
        double frPower = this.drive + this.strafe + turn;
        double blPower = this.drive + this.strafe - turn;
        double brPower = this.drive - this.strafe + turn;
        double maxPower = Math.abs(Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.max(Math.abs(blPower), Math.abs(brPower))));
        
        if (maxPower > 1) {
            flPower = flPower / maxPower;
            frPower = frPower / maxPower;
            blPower = blPower / maxPower;
            brPower = brPower / maxPower;
        }
        this.frontLeft.setPower(flPower);
        this.frontRight.setPower(frPower);
        this.backLeft.setPower(blPower);
        this.backRight.setPower(brPower);
    }
    

//    public double adjustedTicks(){
//        double measuredTicks = ((Math.abs(frontRight.getCurrentPosition()) + Math.abs(frontLeft.getCurrentPosition()) + Math.abs(backRight.getCurrentPosition()) + Math.abs(backLeft.getCurrentPosition())) / 4.0);
//        double angleAdjustment = (Math.abs(Math.abs(drive) - Math.abs(strafe))/power -1) * -1;
//        return Math.sqrt(Math.pow(measuredTicks, 2) + Math.pow(measuredTicks * angleAdjustment, 2));
//    }
//
//    public void strafe(double heading, double strafeAngle, double targetPower, double startPower, double endPower, double distance){
//        distance = Math.abs(distance);
//        startPower = Math.abs(startPower);
//        targetPower = Math.abs(targetPower);
//        endPower = Math.abs(endPower);
//
//        double remainingDistance = distance - adjustedTicks();
//        double acceleratePower = Math.sqrt(ACCEL_RATE * (adjustedTicks() + (1000 * Math.pow(startPower, 2)) + 1));
//        double deceleratePower = Math.sqrt(ACCEL_RATE * (remainingDistance + (1000 * Math.pow(endPower, 2))));
//
//        double currentPower = Math.min(Math.min(acceleratePower, deceleratePower), targetPower);
//        double drive = (Math.cos((strafeAngle - gyro.getRawAngle()) * (Math.PI / 180)));
//        double strafe = (Math.sin((strafeAngle - gyro.getRawAngle()) * (Math.PI / 180)));
//        double turn = ((heading - gyro.getRawAngle()) * .003);
//
//        setPower(drive, strafe, turn, currentPower);
//
//    }
//
//    public void strafe(double heading, double strafeAngle, double targetPower, double startPower){
//
//        startPower = Math.abs(startPower);
//        targetPower = Math.abs(targetPower);
//
//        double acceleratePower = Math.sqrt(ACCEL_RATE * (adjustedTicks() + (1000 * Math.pow(startPower, 2)) + 1));
//
//        double currentPower = Math.min(acceleratePower, targetPower);
//        double drive = (Math.cos((strafeAngle - gyro.getRawAngle()) * (Math.PI / 180)));
//        double strafe = (Math.sin((strafeAngle - gyro.getRawAngle()) * (Math.PI / 180)));
//        double turn = ((heading - gyro.getRawAngle()) * .003);
//
//        setPower(drive, strafe, turn, currentPower);
//
//    }
//    public void resetMotors(){
//        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    }

    public void addTelemetryData(){

    }

    public void addTelemetryData(String driveLabel, String strafeLabel, String turnLabel){

    }
    
}
