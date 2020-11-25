package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controller;

@TeleOp(name = "Operator TeleOp", group ="TeleOp")
//@Disabled
public class OperatorTeleOp extends OpMode {

    private enum FeederState {
        STATE_IDLE,
        STATE_FEED,
        STATE_RESET,
        STATE_DELAY
    }

    private enum IntakeState {
        STATE_OFF,
        STATE_ON,
        STATE_REVERSE
    }

    private enum TubingState {
        STATE_RETRACTED,
        STATE_DEPLOYED
    }

    private enum GripperState {
        STATE_GRIP,
        STATE_EJECT,
        STATE_OPEN
    }

    private enum ShooterState{
        STATE_OFF,
        STATE_TOP_GOAL,
        STATE_POWER_SHOT
    }

    private enum ArmState {
        STATE_UP,
        STATE_DOWN
    }

    private FeederState currentFeederState = FeederState.STATE_IDLE;
    private IntakeState currentIntakeState = IntakeState.STATE_OFF;
    private TubingState currentTubingState = TubingState.STATE_RETRACTED;
    private GripperState currentGripperState = GripperState.STATE_OPEN;
    private ShooterState currentShooterState = ShooterState.STATE_OFF;
    private ArmState currentArmState = ArmState.STATE_UP;

    private ElapsedTime feederTime = new ElapsedTime();
    private ElapsedTime intakeTime = new ElapsedTime();
    private ElapsedTime tubingTime = new ElapsedTime();
    private ElapsedTime gripperTime = new ElapsedTime();
    private ElapsedTime shooterTime = new ElapsedTime();
    private ElapsedTime armTime = new ElapsedTime();

    private DcMotor shooterOne;
    private DcMotor shooterTwo;
    private DcMotor intakeDrive;

    private Servo feeder;
    private Servo tubingDeploy;
    private Servo gripper;
    private Servo lifter;

    Shooter shooter;
    Intake intake;
    WobbleGripper wobble;
    Controller driver;
    Controller operator;

    @Override
    public void init() {

        shooterOne = hardwareMap.get(DcMotor.class, "shooter_one");
        shooterTwo = hardwareMap.get(DcMotor.class, "shooter_two");
        intakeDrive = hardwareMap.get(DcMotor.class, "intake_drive");

        feeder = hardwareMap.get(Servo.class, "feeder");
        tubingDeploy = hardwareMap.get(Servo.class, "tubing_deploy");
        gripper = hardwareMap.get(Servo.class, "gripper");
        lifter = hardwareMap.get(Servo.class, "lifter");

        shooter  = new Shooter(shooterOne, shooterTwo, feeder);
        intake = new Intake(intakeDrive, tubingDeploy);
        wobble = new WobbleGripper(gripper, lifter);

        driver = new Controller(gamepad1);
        operator = new Controller(gamepad2);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        //Feeder State Machine
        switch (currentFeederState){
            case STATE_IDLE:
                if(operator.square()){
                    newState(FeederState.STATE_FEED);
                }
                break;
            case STATE_FEED:
                if (shooter.isRingFed() || feederTime.seconds() > 0.5){
                    newState(FeederState.STATE_RESET);
                }else{
                    shooter.feedRing();
                }
                break;
            case STATE_RESET:
                if (shooter.isReset()){
                    newState(FeederState.STATE_DELAY);
                }else{
                    shooter.resetFeeder();
                }
                break;
            case STATE_DELAY:
                if(feederTime.seconds()>0.4){
                    newState(FeederState.STATE_IDLE);
                }
                break;
        }

        //Intake State Machine
        switch (currentIntakeState){
            case STATE_OFF:
                if(operator.crossPress()){
                    newState(IntakeState.STATE_ON);
                }else if(operator.circlePress()){
                    newState(IntakeState.STATE_REVERSE);
                }
                else{
                    intake.intakeOff();
                }
                break;
            case STATE_ON:
                if(operator.crossPress()){
                    newState(IntakeState.STATE_OFF);
                }else if(operator.circlePress()){
                    newState(IntakeState.STATE_REVERSE);
                }
                else{
                    intake.intakeOn();
                }
                break;
            case STATE_REVERSE:
                if(operator.circlePress()){
                    newState(IntakeState.STATE_ON);
                }else if(operator.crossPress()){
                    newState(IntakeState.STATE_OFF);
                }else{
                    intake.intakeReverse();
                }
                break;
        }

        //Surgical Tubing Roller State Machine
        switch(currentTubingState){
            case STATE_RETRACTED:
                if(operator.down()){
                    newState(TubingState.STATE_DEPLOYED);
                }else{
                    intake.retractTubing();
                }
                break;
            case STATE_DEPLOYED:
                if(operator.up()){
                    newState(TubingState.STATE_RETRACTED);
                }else{
                    intake.deployTubing();
                }
                break;
        }

        //Wobble Goal Gripper State Machine
        switch(currentGripperState){
            case STATE_OPEN:
                if(operator.rightBumper()){
                    newState(GripperState.STATE_GRIP);
                }else if(operator.leftBumper()) {
                    newState(GripperState.STATE_EJECT);
                }else{
                    wobble.open();
                }
                break;
            case STATE_GRIP:
                if(operator.rightBumper()){
                    newState(GripperState.STATE_OPEN);
                }else if(operator.leftBumper()) {
                    newState(GripperState.STATE_EJECT);
                }else{
                    wobble.grip();
                }
                break;
            case STATE_EJECT:
                if(wobble.isEjected()){
                    newState(GripperState.STATE_OPEN);
                }else{
                    wobble.eject();
                }
                break;
        }

        //Shooter State Machine
        switch(currentShooterState){
            case STATE_OFF:
                if(operator.trianglePress()){
                    newState(ShooterState.STATE_TOP_GOAL);
                }else{
                    shooter.setPower(0);
                }
                break;
            case STATE_TOP_GOAL:
                if(operator.left()){
                    newState(ShooterState.STATE_POWER_SHOT);
                }else if(operator.trianglePress()){
                    newState(ShooterState.STATE_OFF);
                }else{
                    shooter.topGoal();
                }
                break;
            case STATE_POWER_SHOT:
                if(operator.right()){
                    newState(ShooterState.STATE_TOP_GOAL);
                }else if(operator.trianglePress()){
                    newState(ShooterState.STATE_OFF);
                }else{
                    shooter.powerShot();
                }
                break;
        }

    }

    private void newState(FeederState newState){
        currentFeederState = newState;
        feederTime.reset();
    }

    private void newState(IntakeState newState){
        currentIntakeState = newState;
        intakeTime.reset();
    }

    private void newState(TubingState newState){
        currentTubingState = newState;
        tubingTime.reset();
    }

    private void newState(GripperState newState){
        currentGripperState = newState;
        gripperTime.reset();
    }

    private void newState(ShooterState newState){
        currentShooterState = newState;
        shooterTime.reset();
    }

    private void newState(ArmState newState){
        currentArmState = newState;
        armTime.reset();
    }
}
