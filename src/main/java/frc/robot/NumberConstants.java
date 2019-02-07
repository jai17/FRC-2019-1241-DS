package frc.robot;

public class NumberConstants {

    /*********************** DRIVETRAIN ************************************/
    // PID constants
    // driving constants
    public static final double pTalonDrive = 0.0;
    public static final double iTalonDrive = 0.0;
    public static final double dTalonDrive = 0.0;
    public static final double fTalonDrive = 0.0;

    public static final double pDrive = 0.0;
    public static final double iDrive = 0.0;
    public static final double dDrive = 0.0;
    public static final double fDrive = 0.0;

    // turning constants
    public static final double pTurn = 0.0;
    public static final double iTurn = 0.0;
    public static final double dTurn = 0.0;

    // gyro constants
    public static final double pGyro = 0.025;
    public static final double iGyro = 0.0;
    public static final double dGyro = 0.001;

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
    public static final double pTalonElevator = 0.0;
    public static final double iTalonElevator = 0.0;
    public static final double dTalonElevator = 0.0;
    public static final double fTalonElevator = 0.0;

    public static final double pElevator = 0.0;
    public static final double iElevator = 0.0;
    public static final double dElevator = 0.0;
    public static final double fElevator = 0.0;

    // MotionMagic constants
    public static final int ELEVATOR_MAX_SPEED = 0;
    public static final int ELEVATOR_CRUISE_VELOCITY = 0;
    public static final int ELEVATOR_ACCELERATION = 0;

    // Position setpoints
    public static final double ELEVATOR_REST_POSITION = 1; 
    public static final double ELEVATOR_LOW_POSITION = 5.5; 
    public static final double ELEVATOR_MID_POSITION = 19.5; 
    public static final double ELEVATOR_HIGH_POSITION = 33; 
    public static final double ELEVATOR_CARGOSHIP_POSITION = 10; 

    /*********************** CARGO ************************************/
    // PID constants
    public static final double pTalonCargo = 0.0;
    public static final double iTalonCargo = 0.0;
    public static final double dTalonCargo = 0.0;
    public static final double fTalonCargo = 0.0;

    public static final double pCargo = 0.0;
    public static final double iCargo = 0.0;
    public static final double dCargo = 0.0;
    public static final double fCargo = 0.0;

    // MotionMagic constants
    public static final int CARGO_MAX_SPEED = 0;
    public static final int CARGO_CRUISE_VELOCITY = 0;
    public static final int CARGO_ACCELERATION = 0;

    // Angle constants 
    public static final double CARGO_FEEDING_ANGLE = 5; 
    public static final double CARGO_INTAKING_ANGLE = 90; 

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