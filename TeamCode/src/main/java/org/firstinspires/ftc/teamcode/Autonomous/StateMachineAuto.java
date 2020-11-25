package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClasses.MecanumControl;
import org.firstinspires.ftc.utilities.Utils;

@TeleOp(name = "State Machine Test", group="Autp")

public class StateMachineAuto extends OpMode {

    private enum State {
        STATE_INITIAL,
        STATE_LOWER_ARM,
        STATE_GRAB_GOAL,
        STATE_VISION,
        STATE_A,
        STATE_B,
        STATE_C
    }

    private enum SwitchState {
        STATE_INITIAL,
        STATE_DELIVER_FIRST,
        STATE_EJECT_FIRST_GOAL,
        STATE_DELIVER_SECOND
    }

    private enum FeederState {
        STATE_IDLE,
        STATE_FEED,
        STATE_RESET,
        STATE_DELAY
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

    private State currentState = State.STATE_INITIAL;
    private SwitchState currentSwitchState = SwitchState.STATE_INITIAL;
    private FeederState currentFeederState = FeederState.STATE_IDLE;
    private GripperState currentGripperState = GripperState.STATE_OPEN;
    private ShooterState currentShooterState = ShooterState.STATE_OFF;
    private ArmState currentArmState = ArmState.STATE_UP;

    public ElapsedTime Runtime = new ElapsedTime();
    private ElapsedTime stateTime = new ElapsedTime();
    private ElapsedTime switchTime = new ElapsedTime();
    private ElapsedTime feederTime = new ElapsedTime();
    private ElapsedTime gripperTime = new ElapsedTime();
    private ElapsedTime shooterTime = new ElapsedTime();
    private ElapsedTime armTime = new ElapsedTime();

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor shooterOne;
    private DcMotor shooterTwo;
    private DcMotor intakeDrive;

    private Servo feeder;
    private Servo tubingDeploy;
    private Servo gripper;
    private Servo lifter;

    MecanumControl robot;
    Shooter shooter;
    Intake intake;
    WobbleGripper wobble;

    @Override
    public void init() {
        Utils.setHardwareMap(hardwareMap);
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        shooterOne = hardwareMap.get(DcMotor.class, "shooter_one");
        shooterTwo = hardwareMap.get(DcMotor.class, "shooter_two");
        intakeDrive = hardwareMap.get(DcMotor.class, "intake_drive");

        feeder = hardwareMap.get(Servo.class, "feeder");
        tubingDeploy = hardwareMap.get(Servo.class, "tubing_deploy");
        gripper = hardwareMap.get(Servo.class, "gripper");
        lifter = hardwareMap.get(Servo.class, "lifter");

        robot = new MecanumControl(frontLeft, frontRight, backLeft, backRight);

        shooter  = new Shooter(shooterOne, shooterTwo, feeder);
        intake = new Intake(intakeDrive, tubingDeploy);
        wobble = new WobbleGripper(gripper, lifter);

        telemetry.addData("Status", "Initialized");


    }

    public void init_loop(){

        wobble.armUp();
        shooter.off();


    }

    public void start(){
        Runtime.reset();
        stateTime.reset();
        feederTime.reset();
        gripperTime.reset();
        shooterTime.reset();
    }

    @Override
    public void loop() {

        switch (currentState) {
            case STATE_INITIAL:

                newState(State.STATE_VISION);

                break;

            case STATE_LOWER_ARM:
                if(wobble.isArmDown()){
                    newState(State.STATE_GRAB_GOAL);
                }else{
                    newState(ArmState.STATE_DOWN);
                }
                break;

            case STATE_GRAB_GOAL:
                if(gripperTime.seconds() > .5){
                }
                newState(GripperState.STATE_GRIP);
                break;

//            case STATE_VISION:
//                if(vision.noRings()){
//                    newState(State.STATE_A);
//                }else if(vision.oneRing()) {
//                    newState(State.STATE_B);
//                }else if(vision.fourRings()){
//                    newState(State.STATE_C);
//                }else{
//
//                }
//                break;

            case STATE_A:
                switch (currentSwitchState){
                    case STATE_INITIAL:
                        newState(SwitchState.STATE_DELIVER_FIRST);
                    case STATE_DELIVER_FIRST:

                }
            case STATE_B:

            case STATE_C:

        }

        //Feeder State Machine
        switch (currentFeederState){
            case STATE_IDLE:
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

        //Wobble Goal Gripper State Machine
        switch(currentGripperState){
            case STATE_OPEN:
                wobble.open();
                break;
            case STATE_GRIP:
                wobble.grip();
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
                shooter.setPower(0);
                break;
            case STATE_TOP_GOAL:
                shooter.topGoal();
                break;
            case STATE_POWER_SHOT:
                shooter.powerShot();
                break;
        }

    }

    private void newState(State newState){
        if (currentState != newState) {
            currentState = newState;
            stateTime.reset();
        }
    }

    private void newState(SwitchState newState){
        if (currentSwitchState != newState) {
            currentSwitchState = newState;
            switchTime.reset();
        }
    }

    private void newState(FeederState newState){
        if (currentFeederState != newState) {
            currentFeederState = newState;
            feederTime.reset();
        }
    }

    private void newState(GripperState newState){
        if (currentGripperState != newState) {
            currentGripperState = newState;
            gripperTime.reset();
        }
    }

    private void newState(ShooterState newState){
        if (currentShooterState != newState) {
            currentShooterState = newState;
            shooterTime.reset();
        }
    }

    private void newState(ArmState newState){
        if (currentArmState != newState) {
            currentArmState = newState;
            armTime.reset();
        }
    }

}
