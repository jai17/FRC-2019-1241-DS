package frc.robot;

public class ElectricalConstants {
    // **************************************************************************
    // ****************************** DRIVETRAIN *******************************
    // **************************************************************************

    // motors (front master, middle & back slaves)
    public static final int LEFT_DRIVE_FRONT = 0;
    public static final int LEFT_DRIVE_MIDDLE = 1;
    public static final int LEFT_DRIVE_BACK = 2;

    public static final int RIGHT_DRIVE_FRONT = 3;
    public static final int RIGHT_DRIVE_MIDDLE = 4;
    public static final int RIGHT_DRIVE_BACK = 5;

    // solenoid
    public static final int SHIFTER_SOLENOID_HIGH = 5;
    public static final int SHIFTER_SOLENOID_LOW = 6;

    // encoder constants
    public static final double ENCODER_GEAR_RATIO = 1.0;
    public static final double WHEEL_RADIUS = 2.0;
    public static final double ROTATIONS_TO_INCHES = 2 * Math.PI * WHEEL_RADIUS * ENCODER_GEAR_RATIO;
    public static final double DRIVE_TO_INCHES = ROTATIONS_TO_INCHES / 4096;

    // **************************************************************************
    // ******************************* ELEVATOR *********************************
    // **************************************************************************

    // motors (right master, left slave)
    public static final int LEFT_ELEVATOR_MOTOR = 6;
    public static final int RIGHT_ELEVATOR_MOTOR = 7;

    // encoder constants
    public static final double ELEVATOR_TO_INCHES = 0.0;

    // *************************************************************************
    // ****************************** CARRIAGE *********************************
    // *************************************************************************

    // motors
    public static final int FEEDER_MOTOR = 8;
    public static final int SHOOTER_MOTOR_LEFT = 9;
    public static final int SHOOTER_MOTOR_RIGHT = 10;

    // solenoids
    public static final int CONE_SOLENOID = 1;
    public static final int EJECTOR_SOLENOID = 2;
    public static final int SLIDER_SOLENOID = 3;

    // digital sensors
    public static final int HATCH_PANEL_DETECTOR_CARRIAGE = 1;
    public static final int CARGO_DETECTOR_CARRIAGE = 6;


    // *************************************************************************
    // **************************** CARGO INTAKE *******************************
    // *************************************************************************

    // motors (right master, left slave)
    public static final int CARGO_ROLLER_MOTOR = 11;
    public static final int CARGO_PIVOT_RIGHT = 12;
    public static final int CARGO_PIVOT_LEFT = 13;

    // digital sensors
    public static final int CARGO_LIMIT_SWITCH = 2;
    public static final int CARGO_OPTICAL = 5; 

    // encoder constants
    public static final double CARGO_TO_DEGREES = 0.0;

    // *************************************************************************
    // **************************** HATCH INTAKE *******************************
    // *************************************************************************

    // motors
    public static final int HATCH_PIVOT_MOTOR = 14;
    public static final int HATCH_ROLLER = 15;

    // digital sensors
    public static final int HATCH_LIMIT_SWITCH = 3;
    public static final int HATCH_PANEL_DETECTOR_INTAKE = 4;

    // encoder constants
    public static final double HATCH_TO_DEGREES = 0.0;
}