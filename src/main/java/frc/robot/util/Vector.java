package frc.robot.util; 

public class Vector {
	private double x;
	private double y;
	private double r;
	private double theta;

	// Use this
	/*
	 * @param r The hypotenuse of the Vector
	 * @param theta The angle of the Vector
	 */
	public Vector(double r, double theta) {
		this.x = r*Math.cos(Math.toRadians(theta));
		this.y = r*Math.sin(Math.toRadians(theta));
		this.r = r;
		this.theta = theta;
	}
	
	public static Vector fromRect(double x, double y) {
		if(x != 0){
			double theta = Math.toDegrees(Math.atan(y/x));
			return new Vector(Math.sqrt(x*x + y*y), theta);
		}else{
			return new Vector(y, 0);
		}
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}
	
	public double getTheta() {
		return theta;
	}
	
	public double getR() {
		return r;
	}

	public Vector(Vector clone) {
		this.x = clone.x;
		this.y = clone.y;
		this.r = clone.r;
		this.theta = clone.theta;
	}

	public Vector add(Vector b) {
		return fromRect(this.x + b.x, this.y + b.y);
	}

	public Vector sub(Vector b) {
		return fromRect(this.x - b.x, this.y - b.y);
	}

	public Vector scalarMult(double c) {
		return fromRect(this.x * c, this.y * c);
	}

	public static Vector fromAngle(double c) {
		c = Math.toRadians(c);
		return fromRect(Math.cos(c), Math.sin(c));
	}

	public Vector unit() {
		double magnitude = this.mag();
		return fromRect(this.x / magnitude, this.y / magnitude);
	}

	public double magSq() {
		return this.dot(this);
	}

	public double dot(Vector b) {
		return this.x * b.x + this.y * b.y;
	}

	public double mag() {
		return Math.sqrt(this.magSq());
	}

	public double scalarProjectOnto(Vector b) {
		return this.dot(b) / b.mag();
	}

	public Vector projectOnto(Vector b) {
		return b.scalarMult(this.dot(b) / b.magSq());
	}

	public Vector rotate(double degrees) {
		double angle = Math.toRadians(degrees);
		double x = this.x * Math.cos(angle) - this.y * Math.sin(angle);
		double y = this.x * Math.sin(angle) + this.y * Math.cos(angle);
		return fromRect(x, y);
	}

	public String toString() {
		return String.format("(%.2f , %.2f)", this.x, this.y);
	}
}