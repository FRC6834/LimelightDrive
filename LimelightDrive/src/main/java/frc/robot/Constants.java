package frc.robot;

public class Constants {
    
    //Drive speeds and turn speeds are represented as percentages (0.5 would be 50% speed)
    public static double fMaxSpeed = 0.5;
    public static double rMaxSpeed = -0.3;
    public static double fMaxTurn = -0.8;
    public static double rMaxTurn = 0.5;

    //Deadband applies to all drive speeds and turns
    public static double driveDeadband = 0.05;

    //CAN IDs for drivetrain
    public static int frontLeftID = 1;
    public static int rearLeftID = 2;
    public static int frontRightID = 3;
    public static int rearRightID = 4;        
}