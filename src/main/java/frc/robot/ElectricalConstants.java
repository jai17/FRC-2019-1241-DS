package frc.robot;

public class ElectricalConstants {
    // **************************************************************************
    // ****************************** DRIVETRAIN *******************************
    // **************************************************************************

    // motors (front master, middle & back slaves)
    public static final int LEFT_DRIVE_FRONT = 2;
    public static final int LEFT_DRIVE_MIDDLE = 3;
    // public static final int LEFT_DRIVE_BACK = 1;

    public static final int RIGHT_DRIVE_FRONT = 16;
    public static final int RIGHT_DRIVE_MIDDLE = 14;
    // public static final int RIGHT_DRIVE_BACK = 15;

    // solenoid
    public static final int SHIFTER_SOLENOID_HIGH = 6;
    public static final int SHIFTER_SOLENOID_LOW = 7;

    // encoder constants
    public static final int RIGHT_ENCODER_A = 4;
    public static final int RIGHT_ENCODER_B = 5;

    public static final int LEFT_ENCODER_A = 2;
    public static final int LEFT_ENCODER_B = 3;

    public static final boolean RIGHT_ENCODER_REVERSE = false;
    public static final boolean LEFT_ENCODER_REVERSE = false;

    public static final double ENCODER_GEAR_RATIO = 1.0;
    public static final double WHEEL_RADIUS = 2.0;
    public static final double ROTATIONS_TO_INCHES = 2 * Math.PI * WHEEL_RADIUS * ENCODER_GEAR_RATIO;
    public static final double DRIVE_TO_INCHES = ROTATIONS_TO_INCHES / 4096;
    public static final double TICKS_PER_INCH =  308;//313.833
 

    // **************************************************************************
    // ******************************* ELEVATOR *********************************
    // **************************************************************************

    // motors (right master, left slave)
    public static final int LEFT_ELEVATOR_MOTOR = 11;
    public static final int RIGHT_ELEVATOR_MOTOR = 4;

    // encoder constants
    public static final double ELEVATOR_TO_INCHES = 408.65;

    // *************************************************************************
    // ****************************** CARRIAGE *********************************
    // *************************************************************************

    // motors
    public static final int FEEDER_MOTOR = 7;
    public static final int SHOOTER_MOTOR_LEFT = 5;
    public static final int SHOOTER_MOTOR_RIGHT = 6;

    // solenoids
    public static final int CLAW_SOLENOID_A = 4;
    public static final int CLAW_SOLENOID_B = 5;
    public static final int EJECTOR_SOLENOID_A = 0;
    public static final int EJECTOR_SOLENOID_B = 1;
    public static final int SLIDER_SOLENOID_A = 2;
    public static final int SLIDER_SOLENOID_B = 3;

    // digital sensors
    public static final int HATCH_PANEL_DETECTOR_TRIGGER_LEFT = 8;
    public static final int HATCH_PANEL_DETECTOR_ECHO_LEFT = 9;

    public static final int HATCH_PANEL_DETECTOR_TRIGGER_RIGHT = 6;
    public static final int HATCH_PANEL_DETECTOR_ECHO_RIGHT = 7;

    public static final int CARGO_DETECTOR_CARRIAGE = 0;


    // *************************************************************************
    // **************************** CARGO INTAKE *******************************
    // *************************************************************************

    // motors (right master, left slave)
    public static final int CARGO_ROLLER_MOTOR = 8;
    public static final int CARGO_PIVOT_RIGHT = 9;
    public static final int CARGO_PIVOT_LEFT = 10;

    // digital sensors
    //public static final int CARGO_LIMIT_SWITCH = 8;
    public static final int CARGO_OPTICAL = 1; 

    // encoder constants
    public static final double CARGO_TO_DEGREES = 1.0;

    // *************************************************************************
    // **************************** HATCH INTAKE *******************************
    // *************************************************************************

    // motors
    public static final int HATCH_PIVOT_MOTOR = 13;
    public static final int HATCH_ROLLER = 12;

    // digital sensors
    //public static final int HATCH_LIMIT_SWITCH = 7;
    //public static final int HATCH_PANEL_DETECTOR_INTAKE = 9;

    // encoder constants
    public static final double HATCH_TO_DEGREES = 1.0;
}