package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.PID;
import org.firstinspires.ftc.teamcode.RingBuffer;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class MecanumControl {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private PID telePID = new PID(.02, 0.0000, .000, 20);
    private PID autoPID = new PID(.03, 0.0000, .000, 20);
    private RingBuffer<Double> timeRing = new RingBuffer<>(20, 0.0);

    private Gyro gyro;

    private double drive, strafe, turn, power, targetAngle, bigTurn;
    private double closestTarget = 0;
    private double previousTarget = 0;

    private final static double ACCEL_RATE = 0.001;
    private static final double TELE_ACCEL = .00015;

    public MecanumControl(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, Gyro gyro) {
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        
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
        if(previousTarget != targetAngle) {
            double currentAngle = gyro.getRawAngle();
            double modCurrentAngle = currentAngle % 360;
            double modTargetAngle = targetAngle % 360;
            double adjTargetAngle;
    
            if (modTargetAngle - modCurrentAngle > 180) {
                adjTargetAngle = modTargetAngle - 360;
            } else if (modTargetAngle - modCurrentAngle < -180) {
                adjTargetAngle = modTargetAngle + 360;
            } else {
                adjTargetAngle = modTargetAngle;
            }
            closestTarget = currentAngle - modCurrentAngle + adjTargetAngle;
        }
        previousTarget = targetAngle;

        return closestTarget;
    }

    public void setPowerTele(double drive, double strafe, double turn, double power){
        this.drive = drive * power;
        this.strafe = strafe * power;
        double currentTime = System.currentTimeMillis();
        double deltaTime = currentTime - timeRing.getValue(currentTime);
        
        if(bigTurn > -0.05) {
            bigTurn -= TELE_ACCEL * deltaTime;
            targetAngle = gyro.getRawAngle();
        }
        else if(bigTurn < 0.05) {
            bigTurn += TELE_ACCEL * deltaTime;
            targetAngle = gyro.getRawAngle();
        }
        if (turn != 0) {
            bigTurn = turn;
            this.turn = turn * power;
        }else {
            this.turn = telePID.update(targetAngle - gyro.getRawAngle()) * Math.abs(power);
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
        turn = autoPID.update(targetAngle - gyro.getRawAngle()) * Math.abs(power);
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
    

    public double adjustedTicks() {
        double measuredTicks = ((Math.abs(frontRight.getCurrentPosition()) + Math.abs(frontLeft.getCurrentPosition()) + Math.abs(backRight.getCurrentPosition()) + Math.abs(backLeft.getCurrentPosition())) / 4.0);
        double angleAdjustment = (Math.abs(Math.abs(drive) - Math.abs(strafe)) / power - 1) * -1;
        return Math.sqrt(Math.pow(measuredTicks, 2) + Math.pow(measuredTicks * angleAdjustment, 2));
    }
    
    public void gyroSteering(double targetAngle, double power){
        
        setPowerAuto(power,0, autoPID.update(targetAngle - gyro.getRawAngle()), 1);
    }
    
    public void resetMotors(){
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void addDriveData(){
        telemetry.addData("drive = ", drive);
        telemetry.addData("strafe = ", strafe);
        telemetry.addData("turn = ", turn);
        telemetry.addData("angle = ", gyro.getRawAngle());
    }

    public void addTelemetryData(String driveLabel, String strafeLabel, String turnLabel){

    }
    
}
