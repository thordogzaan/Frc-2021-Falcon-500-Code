package frc.robot;

// Default
import edu.wpi.first.wpilibj.TimedRobot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// Motor
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;


// Controller
import edu.wpi.first.wpilibj.Joystick;


// Sensor
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;

// encoder
import com.ctre.phoenix.sensors.CANCoder;


// Other 
import java.lang.Math;
import edu.wpi.first.wpilibj.Timer;



public class Robot extends TimedRobot {


  // Joysticks
  private Joystick joystick;
  private Joystick xbox;
  
  
  // Motors
  private WPI_TalonFX[] left  = new WPI_TalonFX[2];
  private WPI_TalonFX[] right = new WPI_TalonFX[2];

  private WPI_TalonSRX turretIntakeLeft;
  private WPI_TalonSRX turretIntakeRight;
  
  private WPI_TalonFX shooterLeft;
  private WPI_TalonFX shooterRight;
  
  /////////////////////////////////
  
  private WPI_TalonSRX mainIntake;

  private WPI_TalonSRX intake_stage1;
  private WPI_TalonSRX intake_stage2;
  
  private WPI_TalonSRX turret;



  // Motor Groupings
  private SpeedControllerGroup lSide;
  private SpeedControllerGroup rSide;
  
  private SpeedControllerGroup turretIntake;
  private SpeedControllerGroup shooter;


  // Drivetrain
  private DifferentialDrive chassis;
  

  
  // Motor IDs (from Phoenix Tuner)
  private int LEFT1_ID = 2;
  private int LEFT2_ID = 3;

  private int RIGHT1_ID = 12;
  private int RIGHT2_ID = 13;
  
  private int MAIN_INTAKE_ID = 1;

  private int INTAKE_STAGE_1_ID = 6;
  private int INTAKE_STAGE_2_ID = 5;

  private int TURRET_INTAKE_LEFT_ID  =  4;
  private int TURRET_INTAKE_RIGHT_ID = 11;

  private int TURRET_ID = 14;

  private int SHOOTER_LEFT_ID  = 0;
  private int SHOOTER_RIGHT_ID = 15;


  // encoders

  //CANCoder _coder = new CANCoder(2);
  //_coder.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, PID_TYPE, DEFAULT_TIMEOUT);



  
  // Joystick buttons and axises
  private int FOREWARD_BACKWARD_AXIS = 1;
  private int LEFT_RIGHT_AXIS        = 2; 

  private int DRIVING_SPEED = 3; // Axis number for the slider thing on the bottom joystick

  private int TRIGGER       = 1; // Button number for the trigger on the joystick
  private int THUMB_BUTTON  = 2;
  private int TOGGLE_CAMERA = 3;
  private int FLASH_BANG    = 4;


  // Xbox buttons and axises
  private int TURRET_AIM_AXIS = 4;

  private int MAIN_INTAKE_AND_STAGE_1_INTAKE_BUTTON    = 1; // Button: A
  private int STAGE_1_INTAKE_AND_STAGE_2_INTAKE_BUTTON = 2; // Button: B
  private int STAGE_2_INTAKE_AND_TURRET_INTAKE_BUTTON  = 4; // Button: Y

  private int ALL_INTAKES_BACKWARD                     = 5; // Button: Left Bumper
  private int ALL_INTAKES_FOREWARD                     = 6; // Button: Right Bumper


  // Input devices
  private NetworkTable limelight;
  private DigitalInput leftTurretLimitSwitch;
  private DigitalInput rightTurretLimitSwitch;


  // Other shit
  private int LEFT_TURRET_LIMIT_SWITCH_PIN_NUMBER  = 0;
  private int RIGHT_TURRET_LIMIT_SWITCH_PIN_NUMBER = 1;

  private double LIMELIGHT_ANGLE = 0.2946857947; // Radians
  private double HEIGHT_FROM_LIMELIGHT_TO_GOAL = 56; // Inches

  private Timer autonomousTimer;

  private double BASE_SPEED_INTAKE = 0.3;

  private double mainIntakeSpeed   = BASE_SPEED_INTAKE * 1.7;
  private double stage1IntakeSpeed = BASE_SPEED_INTAKE* 1.25;
  private double stage2IntakeSpeed = BASE_SPEED_INTAKE * 1.25;
  private double turretIntakeSpeed = BASE_SPEED_INTAKE * 4; // Comes out to 1.2, but is automatically limited by the motor to 1 (i think....)

  private int cameraMode = 0;


  private double DISTANCE_TO_GOAL_FOR_AUTONOMOUS = 185;
  private double MARGIN_OF_ERROR = 5;


  



  @Override
  public void robotInit() {

    joystick = new Joystick(0);
    xbox     = new Joystick(1);

    left[0] = makeTalonFX(LEFT1_ID, false);
    left[1] = makeTalonFX(LEFT2_ID, false);

    right[0] = makeTalonFX(RIGHT1_ID, false);
    right[1] = makeTalonFX(RIGHT2_ID, false);

    lSide = new SpeedControllerGroup(left[0], left[1]);
    rSide = new SpeedControllerGroup(right[0], right[1]);
  
    chassis = new DifferentialDrive(lSide, rSide);

    
  
    mainIntake = makeTalonSRX(MAIN_INTAKE_ID, true);
    
    intake_stage1 = makeTalonSRX(INTAKE_STAGE_1_ID, false);
    intake_stage2 = makeTalonSRX(INTAKE_STAGE_2_ID, false);

    turretIntakeLeft = makeTalonSRX(TURRET_INTAKE_LEFT_ID, false);
    turretIntakeRight = makeTalonSRX(TURRET_INTAKE_RIGHT_ID, false);

    turretIntake = new SpeedControllerGroup(turretIntakeLeft, turretIntakeRight);


    turret = makeTalonSRX(TURRET_ID, false);


    shooterLeft = makeTalonFX(SHOOTER_LEFT_ID, true);
    shooterRight = makeTalonFX(SHOOTER_RIGHT_ID, false);
    
    shooter = new SpeedControllerGroup(shooterLeft, shooterRight);



    leftTurretLimitSwitch = new DigitalInput(LEFT_TURRET_LIMIT_SWITCH_PIN_NUMBER);
    rightTurretLimitSwitch = new DigitalInput(RIGHT_TURRET_LIMIT_SWITCH_PIN_NUMBER);

    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    limelight.getEntry("ledMode").setNumber(1);

    
  }

  private WPI_TalonFX makeTalonFX(int id, boolean invert) { // Creates and configures a TalonFX
    WPI_TalonFX talon = new WPI_TalonFX(id);

    talon.configFactoryDefault();
    talon.setInverted(invert);
    talon.stopMotor();

    return talon;
  }

  private WPI_TalonSRX makeTalonSRX(int id, boolean invert) { // Creates and configures a TalonSRX
    WPI_TalonSRX talon = new WPI_TalonSRX(id);

    talon.configFactoryDefault();
    talon.setInverted(invert);
    talon.stopMotor();

    return talon;
  }

  /********************************************************************************* */
//Micek Section

public void straight(double distance, double speed)
{
  speed = speed * -1;
  right[1].setSelectedSensorPosition(0);
  System.out.println("encoder val"+right[1].getSelectedSensorPosition());
  distance = distance *11900 + right[1].getSelectedSensorPosition();
  System.out.println("distance   "+distance);
  while(distance > right[1].getSelectedSensorPosition())
  {
    chassis.arcadeDrive(speed, 0);
    System.out.println("encoder val"+right[0].getSelectedSensorPosition());
  }
}

public void turn(double degrees, boolean direction, double speed)
{
  boolean RIGHT=true;
  boolean LEFT=false;

  
}
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    //limelight.getEntry("ledMode").setNumber(3);
    //limelight.getEntry("camMode").setNumber(0);
    right[1].setSelectedSensorPosition(0);
    System.out.println("first start" + right[1].getSelectedSensorPosition());
    straight(4,0.48);
  }

  @Override
  public void autonomousPeriodic() {
    
    /*
    aimTurret_automatic();

    mainIntake.set(mainIntakeSpeed);
    intake_stage1.set(stage1IntakeSpeed);
    
    if (getDistanceToGoal() > DISTANCE_TO_GOAL_FOR_AUTONOMOUS + MARGIN_OF_ERROR) {
      chassis.arcadeDrive(-0.5, 0);
    } 
    else if (getDistanceToGoal() < DISTANCE_TO_GOAL_FOR_AUTONOMOUS - MARGIN_OF_ERROR) {
      chassis.arcadeDrive(0.3, 0);
    } 
    else {
      shooter.set(1);
      chassis.arcadeDrive(0, 0);
      intake_stage2.set(stage2IntakeSpeed);
      turretIntake.set(turretIntakeSpeed);
    }

    // chassis.arcadeDrive(-0.8, -0.8);

    */
    
  }
  
  
  private void drive() { // Drives the robot
    double topSpeed = joystick.getRawAxis(DRIVING_SPEED);
    topSpeed += 1;
    topSpeed /= 2;
    topSpeed = 1 - topSpeed;
    chassis.arcadeDrive(joystick.getRawAxis(FOREWARD_BACKWARD_AXIS) * topSpeed, -joystick.getRawAxis(LEFT_RIGHT_AXIS) * 0.66);
  }


  private int limitTurretMotion() { // Stops the turret from turning to far, returns true if the motor was stopped
    if (!leftTurretLimitSwitch.get()) {
      turret.stopMotor();
      return -1;
    } else if (!rightTurretLimitSwitch.get()) {
      turret.stopMotor();
      return 1;
    } else {
      return 0;
    }
  }


  private void aimTurret_manual() { // Manual turret aiming using xbox controller
    double speed = xbox.getRawAxis(TURRET_AIM_AXIS) * 0.2;
    if (limitTurretMotion() == -1) { // If the turret has space to move
      turret.set((speed > 0) ? speed : 0);
    } else if (limitTurretMotion() == 1) {
      turret.set((speed < 0) ? speed : 0);
    } else {
      turret.set(speed);
    }
  }
  
  
  private double getGoalHorizontalOffset() { // Tells how far off the limelight is in the X direction, unit is degrees
    return limelight.getEntry("tx").getDouble(0);
  }

  private double getGoalVerticalOffset() { // Tells how far off the limelight is in the X direction, unit is degrees
    return limelight.getEntry("ty").getDouble(0);
  }
  

  private void aimTurret_automatic() { // Auto aims turret

      double xOffset = getGoalHorizontalOffset(); // Horizontal offset

      if (xOffset < 2 && xOffset > -2) { // If it is within 2 degrees on either side of the goal, dont move it
        turret.stopMotor();
      } else {

        if (limitTurretMotion() != 1) {
          if (xOffset > 10) { // If it is greater than 10 degrees off
            turret.set(0.2);  // Aim towards the goal at 20% power
          } else if (xOffset > 0) { // It is not explicitly stated in the condition, but there is an implied xOffset <= 10, so this is if 0 < xOffset <= 10
            turret.set(0.1);        // Aim towards the goal at 10% power
          }
        }

        if (limitTurretMotion() != -1) {
          if (xOffset < -10) { // This is the same as above, just in the other direction
            turret.set(-0.2);
          } else if (xOffset < 0) {
            turret.set(-0.1);
          }
        } 
      }
  }

  private void aimTurret() {
    aimTurret_manual();
    if (joystick.getRawButton(THUMB_BUTTON)) {
      aimTurret_automatic();
    }
  }


  private double getDistanceToGoal() { // Gets the distance to the goal
    return HEIGHT_FROM_LIMELIGHT_TO_GOAL / Math.tan(LIMELIGHT_ANGLE + Math.toRadians(getGoalVerticalOffset()));
  }

  
  
  private void controlShooter() {
    if (joystick.getRawButton(TRIGGER)) { // If the trigger is pulled (joystick)
      shooter.set(1.25); // Set the speed of the shooter
      
    } else { // If the trigger is not pulled, stop the motor                                                  
      shooter.stopMotor();
    }
  }
  


  private void controlIntakes() {

    if (xbox.getRawButton(ALL_INTAKES_FOREWARD)) { // If the button to turn all intakes on is pressed

      mainIntake.set(mainIntakeSpeed);
      intake_stage1.set(stage1IntakeSpeed);
      intake_stage2.set(stage2IntakeSpeed);
      turretIntake.set(turretIntakeSpeed);

    } else if (xbox.getRawButton(ALL_INTAKES_BACKWARD)) { // If the button to reverse all intakes is pressed

      mainIntake.set(-mainIntakeSpeed);
      intake_stage1.set(-stage1IntakeSpeed);
      intake_stage2.set(-stage2IntakeSpeed);
      turretIntake.set(-turretIntakeSpeed);

    } else {

      if (xbox.getRawButton(MAIN_INTAKE_AND_STAGE_1_INTAKE_BUTTON)) { // Controls the first 2 intakes
        mainIntake.set(mainIntakeSpeed);
        intake_stage1.set(stage1IntakeSpeed);
      } else {
        mainIntake.stopMotor();
      }

      if (xbox.getRawButton(STAGE_1_INTAKE_AND_STAGE_2_INTAKE_BUTTON)) { // Same as above
        intake_stage1.set(stage1IntakeSpeed);
        intake_stage2.set(stage2IntakeSpeed);
      } else {
        if (!xbox.getRawButton(MAIN_INTAKE_AND_STAGE_1_INTAKE_BUTTON)) {
          intake_stage1.stopMotor();
        }
      }

      if (xbox.getRawButton(STAGE_2_INTAKE_AND_TURRET_INTAKE_BUTTON)) {
        intake_stage2.set(stage1IntakeSpeed);
        turretIntake.set(turretIntakeSpeed);
      } else {
        if (!xbox.getRawButton(STAGE_1_INTAKE_AND_STAGE_2_INTAKE_BUTTON)) {
          intake_stage2.stopMotor();
        }
        turretIntake.stopMotor();
      }
    }
  }



  private void checkLimelightMode() {
    if (joystick.getRawButtonPressed(TOGGLE_CAMERA)) {
      if (cameraMode == 0) {
        limelight.getEntry("ledMode").setNumber(1);
        limelight.getEntry("camMode").setNumber(1);
        cameraMode = 1;
      } else {
        limelight.getEntry("ledMode").setNumber(3);
        limelight.getEntry("camMode").setNumber(0);
        cameraMode = 0;
      }
    }
  }


  private void flashBang() {
    if (joystick.getRawButton(FLASH_BANG)) {
      for (int i = 0; i < 100; i++) {
        limelight.getEntry("ledMode").setNumber(3);
        limelight.getEntry("ledMode").setNumber(1);
      }
    }
  }

  private void pointAtGoal() {  
    // NEED ENCODER TO DO THIS PROPERLY
    // Step 1: Set turret to point straight ahead (requires encoder)
    // Step 2: Rotate robot until the value of get getGoalHorizontalOffset() is within an acceptable range
  }


  @Override
  public void teleopInit() {
    chassis.arcadeDrive(0, 0);
    mainIntake.stopMotor();
    intake_stage1.stopMotor();
    intake_stage2.stopMotor();
    turretIntake.stopMotor();
    turret.stopMotor();
    shooter.set(0);
    left[0].setSelectedSensorPosition(0);
    left[1].setSelectedSensorPosition(0);
    right[0].setSelectedSensorPosition(0);
    right[1].setSelectedSensorPosition(0);
  }

  @Override
  public void teleopPeriodic() {
    drive();
    checkLimelightMode();
    controlIntakes();
    aimTurret();
    controlShooter();
    double goal_distance = getDistanceToGoal();
    //System.out.println(goal_distance);
    double degreesl0 = left[0].getSelectedSensorPosition();
    double degreesl1 = left[1].getSelectedSensorPosition();
    double degreesr0 = right[0].getSelectedSensorPosition();
    double degreesr1 = right[1].getSelectedSensorPosition();
    //System.out.println("Left 0 Position:" + degreesl0);
    //System.out.println("Left 1 Postion:" + degreesl1);
    //System.out.println("Right 0 Postion:" + degreesr0);
    System.out.println("Right 1 Postion:" + degreesr1);

    /*
    if (Double.compare(goal_distance, 101)== 0) {
      System.out.println("Correct Shooting Distance");
    }
    else {
      System.out.println("Not Correct Shooting Distance, Aim for about 101.");
    }
    */
    // 175
  }

  @Override
  public void disabledInit() {
    limelight.getEntry("ledMode").setNumber(1);
  }

  @Override
  public void disabledPeriodic() {
    checkLimelightMode();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    flashBang();
  }
}
