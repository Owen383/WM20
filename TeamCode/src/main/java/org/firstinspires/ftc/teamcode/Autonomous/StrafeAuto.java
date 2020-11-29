package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PID;
import org.firstinspires.ftc.teamcode.HardwareClasses.Gyro;
import org.firstinspires.ftc.utilities.IMU;
import org.firstinspires.ftc.utilities.Utils;

@Autonomous(name="StrafeAuto", group="Autonomous Linear Opmode")
//@Disabled
public class StrafeAuto extends LinearOpMode {

    DcMotor fr, fl, br, bl;
    IMU imu;
    private final ElapsedTime runtime = new ElapsedTime();
    private Gyro gyro;
    private PID rotationPID = new PID(.003, 0.0000, .000, 20);


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        initialize();
        waitForStart();
    
        

        turn(10, 1.0);

        strafe(5200, 10, -135, 1.0, 0.0, 0.0);
//        strafe(5200, 10, 180, 1.0, 0.0, 0.0);
//        turn(-75, 1.0);
//        strafe(5200, -75, -90, 1.0, 0.0, 0.0);
//        turn(100, 1.0);
//        strafe(5200, 100, 90, 1.0, 0.0, 0.0);
//        turn(0,1.0);
//
//        strafe(5200, 0, 135,1.0, 0.0, 0.0);
//        strafe(7352, 0, -90,1.0, 0.0, 0.0);
//        turn(45, 1.0);
//        strafe(7352, 45, 0,1.0, 0.0, 0.0);
//        turn(-100, 1.0);
//        strafe(7352, -100, 90,1.0, 0.0, 0.0);
//        turn(0,1.0);
//        strafe(3676, 0, 180,1.0, 0.0, 0.0);
//        strafe(3676, 0, -90,1.0, 0.0, 0.0);
//        turn(2000,1.0);

        setPowerAll(0.0);

    }

    public void initialize(){

        Utils.setHardwareMap(hardwareMap);

        fr = hardwareMap.get(DcMotor.class, "frontright");
        fl = hardwareMap.get(DcMotor.class, "frontleft");
        br = hardwareMap.get(DcMotor.class, "backright");
        bl = hardwareMap.get(DcMotor.class, "backleft");

        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = new IMU("imu");
        gyro = new Gyro(imu, imu.getAngle());
        
        

    }

    public void transition(double heading, double incomingAngle, double incomingPower, double outgoingAngle, double outgoingPower) {

        double accelRate = .00001;

        double drive = Math.cos(incomingAngle - gyro.getRawAngle()) * incomingPower;
        double strafe = Math.sin(incomingAngle - gyro.getRawAngle()) * incomingPower;

        double flPower = drive - strafe;
        double frPower = drive + strafe;
        double blPower = drive + strafe;
        double brPower = drive - strafe;

        double drive2 = Math.cos(outgoingAngle - gyro.getRawAngle()) * outgoingPower;
        double strafe2 = Math.sin(outgoingAngle - gyro.getRawAngle()) * outgoingPower;

        double flOut = drive2 - strafe2;
        double frOut = drive2 + strafe2;
        double blOut = drive2 + strafe2;
        double brOut = drive2 - strafe2;
        boolean bfl = false, bfr = false, bbl = false, bbr = false;

        while (!(bfl && bfr && bbl && bbr)) {

            bfl = flPower + .001 > flOut && flPower - .001 < flOut;
            bfr = frPower + .001 > frOut && frPower - .001 < frOut;
            bbl = blPower + .001 > blOut && blPower - .001 < blOut;
            bbr = brPower + .001 > brOut && brPower - .001 < brOut;

            if (flPower < flOut) {
                flPower += accelRate;
            } else if (flPower > flOut) {
                flPower -= accelRate;
            }

            if (frPower < frOut) {
                frPower += accelRate;
            } else if (frPower > frOut) {
                frPower -= accelRate;
            }

            if (blPower < blOut) {
                blPower += accelRate;
            } else if (blPower > blOut) {
                blPower -= accelRate;
            }

            if (brPower < brOut) {
                brPower += accelRate;
            } else if (brPower > brOut) {
                brPower -= accelRate;
            }

            fl.setPower(flPower);
            fr.setPower(frPower);
            bl.setPower(blPower);
            br.setPower(brPower);

            telemetry.addData("fl power: ", flPower);
    }
    }

    public void turn(double targetAngle, double targetPower){

        targetPower = Math.abs(targetPower);
        double accelRate = 0.02;
        boolean endLoop = false;
        double currentPower;
        double startAngle = gyro.getRawAngle();
        

        if (startAngle > targetAngle) {
            targetPower *= -1;
        }

        while (!endLoop && opModeIsActive()){

            double currentAngle = gyro.getRawAngle();

            double remainingAngle = Math.abs(targetAngle - currentAngle);
            double acceleratePower = Math.sqrt(accelRate * (Math.abs(currentAngle - startAngle) + 1));
            double deceleratePower = Math.sqrt(accelRate * (remainingAngle));
            currentPower = Math.min(Math.min(Math.abs(acceleratePower), Math.abs(deceleratePower)), Math.abs(targetPower));

            if(targetPower < 0){
                endLoop = currentAngle < targetAngle;
                currentPower *= -1;
            }else{
                endLoop = currentAngle > targetAngle;
            }

            telemetry.addData("current angle", currentAngle);
            telemetry.update();



            fl.setPower(currentPower * -1);
            fr.setPower(currentPower);
            bl.setPower(currentPower * -1);
            br.setPower(currentPower);
        }

    }

    public void strafe(double distance, double heading, double strafeAngle, double targetPower, double startPower, double endPower){

        distance = Math.abs(distance);
        startPower = Math.abs(startPower);
        targetPower = Math.abs(targetPower);
        endPower = Math.abs(endPower);

        resetMotors();
        double accelRate = 0.001;
        double currentPower;
        double currentDistance = 0;
        double turn = 0;
        

        while (currentDistance < distance  && opModeIsActive()){
    
            double currentAngle = gyro.getRawAngle();

            double remainingDistance = distance - currentDistance;
            double acceleratePower = Math.sqrt(accelRate * (currentDistance + (1000 * Math.pow(startPower, 2)) + 1));
            double deceleratePower = Math.sqrt(accelRate * (remainingDistance + (1000 * Math.pow(endPower, 2))));

            currentPower = Math.min(Math.min(acceleratePower, deceleratePower), targetPower);
            telemetry.addData("Current Power: ", currentPower);

            double drive = Math.cos(strafeAngle - currentAngle) * currentPower;
            double strafe = Math.sin(strafeAngle - currentAngle) * currentPower;
            turn = rotationPID.update(heading - currentAngle) * Math.abs(currentPower);


            double measuredTicks = ((Math.abs(fr.getCurrentPosition()) + Math.abs(fl.getCurrentPosition()) + Math.abs(br.getCurrentPosition()) + Math.abs(bl.getCurrentPosition())) / 4.0);
            double angleAdjustment = (Math.abs(Math.abs(drive) - Math.abs(strafe))/currentPower -1) * -1;
            currentDistance = Math.sqrt(Math.pow(measuredTicks, 2) + Math.pow(measuredTicks * angleAdjustment, 2));


            double flPower = drive - strafe - turn;
            double frPower = drive + strafe + turn;
            double blPower = drive + strafe - turn;
            double brPower = drive - strafe + turn;
            double maxPower = Math.abs(Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.max(Math.abs(blPower), Math.abs(brPower))));

            if (maxPower > 1) {
                flPower = flPower / maxPower;
                frPower = frPower / maxPower;
                blPower = blPower / maxPower;
                brPower = brPower / maxPower;
            }

            fl.setPower(flPower);
            fr.setPower(frPower);
            bl.setPower(blPower);
            br.setPower(brPower);

            telemetry.addData("Drive: ", drive);
            telemetry.addData("Strafe: ", strafe);
            telemetry.addData("Turn: ", turn);
            telemetry.addData("current angle", currentAngle);
            telemetry.update();

        }

    }

    public void resetMotors(){
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void setPowerAll(double power){

        fr.setPower(power);
        fl.setPower(power);
        br.setPower(power);
        bl.setPower(power);

    }
}
