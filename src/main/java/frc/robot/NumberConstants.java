package frc.robot;

public class NumberConstants {

    /*********************** DRIVETRAIN ************************************/
    // PID constants
    // driving constants
    public static final double pTalonDrive = 0.0;
    public static final double iTalonDrive = 0.0;
    public static final double dTalonDrive = 0.0;
    public static final double fTalonDrive = 0.0;

    public static final double pDrive = 0.2;
    public static final double iDrive = 0.0;
    public static final double dDrive = 0.0;
    public static final double fDrive = 0.1;

    // turning constants
    public static final double pTurn = 0.01;
    public static final double iTurn = 0.0;
    public static final double dTurn = 0.0;

    // gyro constants
    public static final double pGyro = 0.0;
    public static final double iGyro = 0.0;
    public static final double dGyro = 0.0;

    // locking constants (lock drive base when scoring)
    public static final double pLock = 0.0;
    public static final double iLock = 0.0;
    public static final double dLock = 0.0;

    // MotionMagic constants
    public static final int DRIVE_MAX_SPEED = 0;
    public static final int DRIVE_CRUISE_VELOCITY = 0;
    public static final int DRIVE_ACCELERATION = 0;

    /*********************** ELEVATOR ************************************/
    // PID constants
    public static final double pTalonElevator = 0.4;
    public static final double iTalonElevator = 0.0;
    public static final double dTalonElevator = 0.002;
    public static final double fTalonElevator = 0.2;

    public static final double pElevator = 0.0;
    public static final double iElevator = 0.0;
    public static final double dElevator = 0.0;
    public static final double fElevator = 0.0;

    // MotionMagic constants
    public static final int ELEVATOR_MAX_SPEED = 4534;
    public static final int ELEVATOR_CRUISE_VELOCITY = 0;
    public static final int ELEVATOR_ACCELERATION = 0;

    // Position setpoints
    public static final double ELEVATOR_REST_POSITION = 3.5; 
    public static final double HATCH_FINESSE_POSITION = 7.5;
    public static final double ELEVATOR_LOW_HATCH_POSITION = 14; 
    public static final double ELEVATOR_HATCH_FEEDER = 14; 
    public static final double ELEVATOR_MID_HATCH_POSITION = 44; 
    public static final double ELEVATOR_HIGH_HATCH_POSITION = 75.5; 
    public static final double ELEVATOR_CARGOSHIP_POSITION = 10; 

    /*********************** CARGO ************************************/
    // PID constants
    public static final double pTalonCargo = 0.5;
    public static final double iTalonCargo = 0.0;
    public static final double dTalonCargo = 0.005;
    public static final double fTalonCargo = 2.7;

    public static final double pCargo = 0.0;
    public static final double iCargo = 0.0;
    public static final double dCargo = 0.0;
    public static final double fCargo = 0.0;

    // MotionMagic constants
    public static final int CARGO_MAX_SPEED = 460;
    public static final int CARGO_CRUISE_VELOCITY = 460;
    public static final int CARGO_ACCELERATION = 0;

    // Angle constants 
    public static final double CARGO_FEEDING_ANGLE = 150; 
    public static final double CARGO_RESTING_ANGLE = 300; 
    public static final double CARGO_INTAKING_ANGLE = 1700; 

    // Soft Limits
    public static final int CARGO_FORWARD_SOFT_LIMIT = 0;
    public static final int CARGO_BACK_SOFT_LIMIT = 0;

    /*********************** HATCH ************************************/
    public static final double pTalonHatch = 0.0;
    public static final double iTalonHatch = 0.0;
    public static final double dTalonHatch = 0.0;
    public static final double fTalonHatch = 0.0;

    public static final double pHatch = 0.0;
    public static final double iHatch = 0.0;
    public static final double dHatch = 0.0;
    public static final double fHatch = 0.0;

    // Soft Limits
    public static final int HATCH_FORWARD_SOFT_LIMIT = 0;
    public static final int HATCH_BACK_SOFT_LIMIT = 0;

    // MotionMagic constants
    public static final int HATCH_MAX_SPEED = 0;
    public static final int HATCH_CRUISE_VELOCITY = 0;
    public static final int HATCH_ACCELERATION = 0;

    // Angle constants 
    public static final double HATCH_FEEDING_ANGLE = 0; 
    public static final double HATCH_INTAKING_ANGLE = 90; 
    public static final double HATCH_REST_ANGLE = -85; 

    /*********************** CARRIAGE ************************************/

}