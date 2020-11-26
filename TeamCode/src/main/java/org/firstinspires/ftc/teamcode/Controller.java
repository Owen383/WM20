package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller {


    private Gamepad gamepad;
    private int iSq;
    private int iTr;
    private int iCr;
    private int iCi;
    private int jSq;
    private int jTr;
    private int jCr;
    private int jCi;
    private int jBL;
    private int jBR;
    private boolean toggleSq = false;
    private boolean toggleTr = false;
    private boolean toggleCr = false;
    private boolean toggleCi = false;
    private boolean pressSq = false;
    private boolean pressTr = false;
    private boolean pressCr = false;
    private boolean pressCi = false;
    private boolean pressBL = false;
    private boolean pressBR = false;

    public Controller(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public Thumbstick getRightThumbstick() {
        return new Controller.Thumbstick(gamepad.right_stick_x, gamepad.right_stick_y);
    }

    public Thumbstick getLeftThumbstick() {
        return new Controller.Thumbstick(gamepad.left_stick_x, gamepad.left_stick_y);
    }

    public boolean cross(){
        return gamepad.a;
    }

    public boolean circle(){
        return gamepad.b;
    }

    public boolean square(){
        return gamepad.x;
    }

    public boolean triangle(){
        return gamepad.y;
    }

    public boolean squareToggle(){

        if (square()){ iSq++; }
        else{ iSq = 0; }

        if(iSq == 1){ toggleSq = !toggleSq; }


        return (toggleSq);
    }

    public int getiSq(){
        return iSq;
    }

    public boolean triangleToggle(){

        if (gamepad.y){ iTr++; }
        else{ iTr = 0; }

        if(iTr == 1){ toggleTr = !toggleTr; }

        return (toggleTr);
    }

    public boolean crossToggle(){

        if (gamepad.a){ iCr++; }
        else{ iCr = 0; }

        if(iCr == 1){ toggleCr = !toggleCr; }

        return (toggleCr);
    }

    public boolean circleToggle(){

        if (gamepad.b){ iCi++; }
        else{ iCi = 0; }

        if(iCi == 1){ toggleCi = !toggleCi; }

        return (toggleCi);
    }

    public boolean squarePress(){

        if (gamepad.x){jSq++; }
        else{ jSq = 0; }

        if(jSq == 1){ pressSq = true; }

        return (pressSq);
    }

    public boolean trianglePress(){

        if (gamepad.y){ jTr++; }
        else{ jTr = 0; }

        if(jTr == 1){ pressTr = true; }

        return (pressTr);
    }

    public boolean crossPress(){

        if (gamepad.a){jCr++; }
        else{ jCr = 0; }

        if(jCi == 1){ pressCr = true; }

        return (pressCr);
    }

    public boolean circlePress(){

        if (gamepad.b){ jCi++; }
        else{ jCi = 0; }

        if(jCi == 1){ pressCi = true; }

        return (pressCi);
    }

    public boolean up(){
        return gamepad.dpad_up;
    }

    public boolean down(){
        return gamepad.dpad_down;
    }

    public boolean left(){
        return gamepad.dpad_left;
    }

    public boolean right(){
        return gamepad.dpad_right;
    }

    public boolean leftStick(){
        return gamepad.left_stick_button;
    }

    public boolean rightStick(){
        return gamepad.right_stick_button;
    }

    public boolean leftBumper(){
        return gamepad.left_bumper;
    }

    public boolean rightBumper(){
        return gamepad.right_bumper;
    }

    public boolean leftBumperPress(){

        if (gamepad.left_bumper){ jBL++; }
        else{ jBL = 0; }

        if(jBL == 1){ pressBL = true; }
        else{ pressBL = false; }

        return (pressBL);
    }

    public boolean rightBumperPress(){

        if (gamepad.right_bumper){ jBR++; }
        else{ jBR = 0; }

        if(jBR == 1){ pressBR = true; }
        else{ pressBR = false; }

        return (pressBR);
    }

    public float leftTrigger(){
        return gamepad.left_trigger;
    }

    public float rightTrigger(){
        return gamepad.right_trigger;
    }

    public class Thumbstick {

        private double rawX;
        private double rawY;
        private double shiftedX;
        private double shiftedY;

        public Thumbstick(Double x, Double y) {
            this.rawX = x;
            this.rawY = y;
        }

        public Thumbstick(Float x, Float y) {
            this.rawX = x;
            this.rawY = y;
        }

        public double getX() {
            return rawX;
        }

        public double getY() {
            return rawY;
        }

        public void setShift(double shiftAngle) {
            this.shiftedX = (this.rawX * Math.cos(Math.toRadians(shiftAngle))) - (this.rawY * Math.sin(Math.toRadians(shiftAngle)));
            this.shiftedY = (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle)));
        }

        public double getShiftedX() {
            return shiftedX;
        }

        public double getShiftedY() {
            return shiftedY;
        }

        public double getShiftedX(Double shiftAngle) {
            return (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle)));
        }

        public double getShiftedY(Double shiftAngle) {
            return (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle)));
        }

        public double getInvertedX() {
            return rawX * -1;
        }

        public double getInvertedY() {
            return rawY * -1;
        }

        public double getInvertedShiftedX() {
            return shiftedX * -1;
        }

        public double getInvertedShiftedY() {
            return shiftedY * -1;
        }

        public double getInvertedShiftedX(Double shiftAngle) {
            return (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle))) * -1;
        }

        public double getInvertedShiftedY(Double shiftAngle) {
            return (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle))) * -1;
        }


    }
}
