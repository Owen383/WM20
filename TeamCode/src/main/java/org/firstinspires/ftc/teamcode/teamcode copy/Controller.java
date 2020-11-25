package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller {


    private Gamepad gamepad;

    public Controller(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public Thumbstick getRightThumbstick() {
        return new Controller.Thumbstick(gamepad.right_stick_x, gamepad.right_stick_y);
    }

    public Thumbstick getLeftThumbstick() {
        return new Controller.Thumbstick(gamepad.left_stick_x, gamepad.left_stick_y);
    }

    public boolean a(){
        return gamepad.a;
    }

    public boolean b(){
        return gamepad.b;
    }

    public boolean x(){
        return gamepad.x;
    }

    public boolean y(){
        return gamepad.y;
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
