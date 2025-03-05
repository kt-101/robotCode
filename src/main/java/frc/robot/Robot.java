/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;

// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.XboxController;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  private static final String kDefaultAuto = "Default";
  private static final String kCentertoCoral = "Center --> Coral";
  private static final String kLefttoCoral = "Left --> Coral";
  private static final String kRighttoCoral = "Right --> Coral";
  private static final String kJustDrive = "Just Drive";
  private static final String kTest = "Test";
  private static final String kTest2 = "Test 2";
  private static final String kTest3 = "Test 3";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final SparkMax ejectMotor = new SparkMax(5, MotorType.kBrushed);

  private final SparkMax leftLeader = new SparkMax(1, MotorType.kBrushed);
  private final SparkMax leftFollower = new SparkMax(2, MotorType.kBrushed);
  private final SparkMax rightLeader = new SparkMax(3, MotorType.kBrushed);
  private final SparkMax rightFollower = new SparkMax(4, MotorType.kBrushed);

  private final DifferentialDrive myDrive = new DifferentialDrive(leftLeader, rightLeader);

  private final SparkMaxConfig driveConfig = new SparkMaxConfig();
  private final SparkMaxConfig ejectConfig = new SparkMaxConfig();

  private final XboxController gamepad0 = new XboxController(0); // driver Controller
  // private final XboxController gamepad1 = new XboxController(1); //operator
  // controller

  // old code
  // private Spark leftMotor1 = new Spark(0);
  // private Spark leftMotor2 = new Spark(1);
  // private Spark rightMotor1 = new Spark(2);
  // private Spark rightMotor2 = new Spark(3);
  // private Spark ejectMotor = new Spark(4);
  // private Joystick joy1 = new Joystick(0);

  private double driveSpeed = 1;
  private final Timer timer1 = new Timer();

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kTest);
    m_chooser.addOption("Right --> Coral", kRighttoCoral);
    m_chooser.addOption("Center --> Coral", kCentertoCoral);
    m_chooser.addOption("Left --> Coral", kLefttoCoral); // add all new auto modes below the setDefaultOption line
    m_chooser.addOption("Just Drive", kJustDrive);
    m_chooser.addOption("Test", kTest);
    m_chooser.addOption("Test 2", kTest2);
    m_chooser.addOption("Test 3", kTest3);
    SmartDashboard.putData("Auto choices", m_chooser);

    driveConfig.smartCurrentLimit(60);
    driveConfig.voltageCompensation(12);

    driveConfig.follow(leftLeader);
    leftFollower.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveConfig.follow(rightLeader);
    rightFollower.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveConfig.disableFollowerMode();
    rightLeader.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveConfig.inverted(true);
    leftLeader.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    ejectConfig.smartCurrentLimit(60);
    ejectConfig.voltageCompensation(10);
    ejectMotor.configure(ejectConfig, ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);

    timer1.start();

  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    timer1.restart();
  }

  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCentertoCoral:
      if(timer1.get() < 4){
        myDrive.tankDrive(.5, .5);
      }else if(timer1.get() < 5){
          myDrive.tankDrive(0, 0);
      }else if(timer1.get() < 7){
        ejectMotor.set(0.35);
        myDrive.tankDrive(0, 0);
      }else{
        myDrive.tankDrive(0, 0);
        ejectMotor.set(0);
      }
        break;
      case kRighttoCoral:
        if(timer1.get() < 8){
          myDrive.tankDrive(.7, .7);
        }else if(timer1.get() < 9){
            myDrive.tankDrive(0, .30);
        }else if(timer1.get() < 11){
          ejectMotor.set(0.35);
          myDrive.tankDrive(0, 0);
        }else{
          myDrive.tankDrive(0, 0);
          ejectMotor.set(0);
        }
        break;
      case kLefttoCoral:
        if(timer1.get() < 8){
          myDrive.tankDrive(.7, .7);
        }else if(timer1.get() < 9){
            myDrive.tankDrive(.3, 0);
        }else if(timer1.get() < 11){
          ejectMotor.set(0.35);
          myDrive.tankDrive(0, 0);
        }else{
          myDrive.tankDrive(0, 0);
          ejectMotor.set(0);
        }
        break;
      case kTest:
      //Center
        if(timer1.get() < 8){
          myDrive.tankDrive(.7, .7);
        }else if(timer1.get() < 9){
            myDrive.tankDrive(0, .30);
        }else if(timer1.get() < 11){
          ejectMotor.set(0.35);
          myDrive.tankDrive(0, 0);
        }else{
          myDrive.tankDrive(0, 0);
          ejectMotor.set(0);
        }
        break;
      case kJustDrive:
        if (timer1.get() < .9) {
          myDrive.tankDrive(.5, .5);
        } else {
          myDrive.tankDrive(0, 0);
        }
        break;
      case kDefaultAuto:
        default:
        break;
    }

  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    if (gamepad0.getLeftBumperButton() == true) {
      driveSpeed = 2; // this will set to half speed, you can divide by other numbers setting this to
                      // 1.5 for example will give 2/3rds speed.
    }

    if (gamepad0.getRightBumperButton()) {
      driveSpeed = 1; // set back to full speed.
    }

    myDrive.arcadeDrive(-gamepad0.getLeftY()/driveSpeed, -gamepad0.getRightX()/driveSpeed);

    if (gamepad0.getXButton()) {
      ejectMotor.set(0.35);
    }else if (gamepad0.getBButton()) {
      ejectMotor.set(0.60);
    }else{
      ejectMotor.set(0);
    }

    // double speed = joy1.getRawAxis(1) * 0.6;
    // double turn = -joy1.getRawAxis(4) * 0.3;

    // double left = speed + turn;
    // double right = speed - turn;

    // leftMotor1.set(left);
    // leftMotor2.set(left);
    // rightMotor1.set(-right);
    // rightMotor2.set(-right);
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}