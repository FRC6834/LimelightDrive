// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

//import required for XBox Controller
import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  // These numbers must be tuned for your Robot!  Be careful!
  private final double STEER_K = 0.03;                    // how hard to turn toward the target
  private final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
  private final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
  private final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;

  //Limelight
  private Limelight cam1 = new Limelight(30, 25);

  //Drivetrain
  RobotDrivetrain drivetrain = new RobotDrivetrain();

  //Motor Controller Object
  //CAN ID remains the same regardless of the motor being tested
  //If a BRUSHED motor is being test, motor type must be changed to kBrushed
  //Follow URL for more info: https://codedocs.revrobotics.com/java/com/revrobotics/cansparkmax
  //private CANSparkMax testMotor = new CANSparkMax(0, MotorType.kBrushless);

  //XBox Controller Object
  //Constructor takes int parameter specifying the USB port that controller is plugged into on the laptop (left side is port 0)
  //Follow URL for more info: https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/XboxController.html
  private XboxController controller = new XboxController(0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  
    cam1.updateValues();
    cam1.camDashDetails();

    aimAndRange();

        double steer = controller.getRightX();
        double drive = controller.getLeftY();
        boolean auto = controller.getAButton();

        steer *= 0.70;
        drive *= 0.70;

        if (auto){
          if (cam1.hasTarget())
          {
                drivetrain.arcadeDrive(m_LimelightDriveCommand,m_LimelightSteerCommand);
          }
          else
          {
                drivetrain.arcadeDrive(0.0,0.0);
          }
        }
        else{
          drivetrain.arcadeDrive(drive,steer);
        }

     //Curvature Drive  
     double forwardSpeed = controller.getRightTriggerAxis();
     double reverseSpeed = controller.getLeftTriggerAxis();
     double turn = controller.getLeftX();
     
     if (forwardSpeed > 0){
       drivetrain.curvatureDrive(speedCalc(forwardSpeed, true), turnCalc(turn, true));
     }
     else if (reverseSpeed > 0){
       drivetrain.curvatureDrive(speedCalc(reverseSpeed, false), turnCalc(turn, false));
     }
     else{
       drivetrain.curvatureDrive(0,0);
     }
 
     //D-Pad controls for fine movements
     int dPad = controller.getPOV(); //scans to see which directional arrow is being pushed
     drivetrain.dPadGetter(dPad);
  }

  //Applies deadband to drive speeds
  public double speedCalc(double speed, boolean forward){
    if(forward){
      return MathUtil.applyDeadband(Constants.fMaxSpeed*speed,Constants.driveDeadband, 0.6);
    }
    return MathUtil.applyDeadband(Constants.rMaxSpeed*speed,Constants.driveDeadband, 0.6);
  }
  
  //Applies deadband to turn speeds
  public double turnCalc (double turn, boolean forward){
    if(forward){
      return MathUtil.applyDeadband(Constants.fMaxTurn*turn, Constants.driveDeadband, 0.6);
    }
    return MathUtil.applyDeadband(Constants.rMaxTurn*turn, Constants.driveDeadband, 0.6);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  //
  public void aimAndRange(){

    if (cam1.hasTarget()){
      m_LimelightDriveCommand = 0.0;
      m_LimelightSteerCommand = 0.0;
    }

    // Start with proportional steering
    double steer_cmd = cam1.getX() * STEER_K;
    m_LimelightSteerCommand = steer_cmd;

    // try to drive forward until the target area reaches our desired area
    double drive_cmd = (DESIRED_TARGET_AREA - cam1.getArea()) * DRIVE_K;

    // don't let the robot drive too fast into the goal
    if (drive_cmd > MAX_DRIVE){
          drive_cmd = MAX_DRIVE;
    }
    m_LimelightDriveCommand = drive_cmd;
  }
}