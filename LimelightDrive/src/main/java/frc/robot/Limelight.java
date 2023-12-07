package frc.robot;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Limelight{
    private double x;
    private double y;
    private double area;
    private boolean hasTarget;
    private NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private NetworkTableEntry tv;

    // how many degrees back is your limelight rotated from perfectly vertical?
    private double limelightMountAngleDegrees;
    // distance from the center of the Limelight lens to the floor
    private double limelightLensHeightInches;
    // distance from the target to the floor
    private double goalHeightInches = 60;   

    public Limelight(double mountAngle, double lensHeight){
        limelightMountAngleDegrees = mountAngle;
        limelightLensHeightInches = lensHeight;
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx"); //horizontal offset
        ty = table.getEntry("ty"); //vertical offset
        ta = table.getEntry("ta"); //Target Area (% of Target)
        tv = table.getEntry("tv"); //0 or 1 - determines if a target is detected
        //Forwards ports so Limelight can be used when USB tethered
        for (int port = 5800; port <= 5807; port++) {
            PortForwarder.add(port, "limelight.local", port);
        }
    }

    public void updateValues(){
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        if(tv.getDouble(0.0) == 1){
            hasTarget = true;
        }
        else{
            hasTarget = false;
        }
        getAngleToGoalDegrees();
        getAngleToGoalRadians();
        getDistanceToGoal();
    }

    public boolean hasTarget(){
        return hasTarget;
    }
    
    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public double getArea(){
        return area;
    }

    public void camDashDetails(){
        //post to smart dashboard periodically
        SmartDashboard.putNumber("X", getX());
        SmartDashboard.putNumber("Y", getY());
        SmartDashboard.putNumber("Area", getArea());
        SmartDashboard.putBoolean("Has Target", hasTarget);
        SmartDashboard.putNumber("Angle to Goal Degrees", getAngleToGoalDegrees());
        SmartDashboard.putNumber("Angle to Goal Radians", getAngleToGoalRadians());
        SmartDashboard.putNumber("Distance to Goal Inches", getDistanceToGoal());
    } 

    //Gets the angle between the limelight and goal in degrees
    public double getAngleToGoalDegrees(){
        return limelightMountAngleDegrees + getY();
    }

    //Gets the angle between the limelight and the goal in radians
    public double getAngleToGoalRadians(){
        return getAngleToGoalDegrees() * (3.14159 / 180.0);
    }

    //Calculates the distance from the limelight to the goal
    public double getDistanceToGoal(){
        return (goalHeightInches - limelightLensHeightInches) / Math.tan(getAngleToGoalRadians());
    }    
}