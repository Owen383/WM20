package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller {
	
	
	private final Gamepad gamepad;
	private boolean isSq;
	private boolean isTr;
	private boolean isCi;
	private boolean isCr;
	private boolean isUp;
	private boolean isLeft;
	private boolean isDown;
	private boolean isRight;
	private boolean isLS;
	private boolean isRS;
	private boolean isLB;
	private boolean isRB;
	private boolean toggleSq = false;
	private boolean toggleTr = false;
	private boolean toggleCr = false;
	private boolean toggleCi = false;
	private boolean toggleUp = false;
	private boolean toggleDown = false;
	private boolean toggleLeft = false;
	private boolean toggleRight = false;
	private boolean toggleLS = false;
	private boolean toggleRS = false;
	private boolean toggleLB = false;
	private boolean toggleRB = false;
	
	
	public Controller(Gamepad gamepad) {
		this.gamepad = gamepad;
	}
	
	public Thumbstick getRightThumbstick() {
		return new Controller.Thumbstick(gamepad.right_stick_x, gamepad.right_stick_y);
	}
	
	public Thumbstick getLeftThumbstick() {
		return new Controller.Thumbstick(gamepad.left_stick_x, gamepad.left_stick_y);
	}
	
	//cross
	public boolean cross() {
		return gamepad.a;
	}
	
	public boolean crossPress() {
		boolean wasCr = isCr;
		return (isCr = gamepad.a) && !wasCr;
	}
	
	public boolean crossToggle() {
		boolean wasCr = isCr;
		if ((isCr = gamepad.a) && !wasCr) {
			toggleCr = !toggleCr;
		}
		return (toggleCr);
	}
	
	
	//circle
	public boolean circle() {
		return gamepad.b;
	}
	
	public boolean circlePress() {
		boolean wasCi = isCi;
		return (isCi = gamepad.b) && !wasCi;
	}
	
	public boolean circleToggle() {
		boolean wasCi = isCi;
		if ((isCi = gamepad.b) && !wasCi) {
			toggleCi = !toggleCi;
		}
		return (toggleCi);
	}
	
	
	//square
	public boolean square() {
		return gamepad.x;
	}
	
	public boolean squarePress() {
		boolean wasSq = isSq;
		return (isSq = gamepad.x) && !wasSq;
	}
	
	public boolean squareToggle() {
		boolean wasSq = isSq;
		if ((isSq = gamepad.x) && !wasSq) {
			toggleSq = !toggleSq;
		}
		return (toggleSq);
	}
	
	
	//triangle
	public boolean triangle() {
		return gamepad.y;
	}
	
	public boolean trianglePress() {
		boolean wasTr = isTr;
		return (isTr = gamepad.y) && !wasTr;
	}
	
	public boolean triangleToggle() {
		boolean wasTr = isTr;
		if ((isTr = gamepad.y) && !wasTr) {
			toggleTr = !toggleTr;
		}
		return (toggleTr);
	}
	
	
	//dpad up
	public boolean up() {
		return gamepad.dpad_up;
	}
	
	public boolean upPress() {
		boolean wasUp = isUp;
		return (isUp = gamepad.dpad_up) && !wasUp;
	}
	
	public boolean upToggle() {
		boolean wasUp = isUp;
		if ((isUp = gamepad.dpad_up) && !wasUp) {
			toggleUp = !toggleUp;
		}
		return (toggleUp);
	}
	
	
	//dpad down
	public boolean down() {
		return gamepad.dpad_down;
	}
	
	public boolean downPress() {
		boolean wasDown = isDown;
		return (isDown = gamepad.dpad_down) && !wasDown;
	}
	
	public boolean downToggle() {
		boolean wasDown = isDown;
		if ((isDown = gamepad.dpad_down) && !wasDown) {
			toggleDown = !toggleDown;
		}
		return (toggleDown);
	}
	
	
	//dpad left
	public boolean left() {
		return gamepad.dpad_left;
	}
	
	public boolean leftPress() {
		boolean wasLeft = isLeft;
		return (isLeft = gamepad.dpad_left) && !wasLeft;
	}
	
	public boolean leftToggle() {
		boolean wasLeft = isLeft;
		if ((isLeft = gamepad.dpad_left) && !wasLeft) {
			toggleLeft = !toggleLeft;
		}
		return (toggleLeft);
	}
	
	
	//dpad right
	public boolean right() {
		return gamepad.dpad_right;
	}
	
	public boolean rightPress() {
		boolean wasRight = isRight;
		return (isRight = gamepad.dpad_right) && !wasRight;
	}
	
	public boolean rightToggle() {
		boolean wasRight = isRight;
		if ((isRight = gamepad.dpad_right) && !wasRight) {
			toggleRight = !toggleRight;
		}
		return (toggleRight);
	}
	
	
	//left stick button
	public boolean LS() {
		return gamepad.left_stick_button;
	}
	
	public boolean LSPress() {
		boolean wasLS = isLS;
		return (isLS = gamepad.left_stick_button) && !wasLS;
	}
	
	public boolean LSToggle() {
		boolean wasLS = isLS;
		if ((isLS = gamepad.left_stick_button) && !wasLS) {
			toggleLS = !toggleLS;
		}
		return (toggleLS);
	}
	
	
	//right stick button
	public boolean RS() {
		return gamepad.right_stick_button;
	}
	
	public boolean RSPress() {
		boolean wasRS = isRS;
		return (isRS = gamepad.right_stick_button) && !wasRS;
	}
	
	public boolean RSToggle() {
		boolean wasRS = isRS;
		if ((isRS = gamepad.right_stick_button) && !wasRS) {
			toggleRS = !toggleRS;
		}
		return (toggleRS);
	}
	
	
	//left bumper
	public boolean LB() {
		return gamepad.left_bumper;
	}
	
	public boolean LBPress() {
		boolean wasLB = isLB;
		return (isLB = gamepad.left_bumper) && !wasLB;
	}
	
	public boolean LBToggle() {
		boolean wasLB = isLB;
		if ((isLB = gamepad.left_bumper) && !wasLB) {
			toggleLB = !toggleLB;
		}
		return (toggleLB);
	}
	
	
	//right bumper
	public boolean RB() {
		return gamepad.right_bumper;
	}
	
	public boolean RBPress() {
		boolean wasRB = isRB;
		return (isRB = gamepad.right_bumper) && !wasRB;
	}
	
	public boolean RBToggle() {
		boolean wasRB = isRB;
		if ((isRB = gamepad.right_bumper) && !wasRB) {
			toggleRB = !toggleRB;
		}
		return (toggleRB);
	}
	
	
	//left trigger
	public float LT() {
		return gamepad.left_trigger;
	}
	
	
	//right trigger
	public float RT() {
		return gamepad.right_trigger;
	}
	
	
	public boolean options() {
		return gamepad.options;
	}
	
	
	public boolean share() {
		return gamepad.share;
	}
	
	
	public class Thumbstick {
		
		private final double rawX;
		private final double rawY;
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
		
		public boolean isInput() {
			return (getX() != 0) || (getY() != 0);
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
