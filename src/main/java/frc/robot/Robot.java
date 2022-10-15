// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.Timer;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
  WPI_TalonSRX liftyleft = new WPI_TalonSRX(5);
  WPI_TalonSRX liftyright = new WPI_TalonSRX(6);

  WPI_TalonSRX shooterlift = new WPI_TalonSRX(8);



  WPI_VictorSPX right_motor_back = new WPI_VictorSPX(4);
  WPI_VictorSPX left_motor_front = new WPI_VictorSPX(2);
  WPI_VictorSPX left_motor_back = new WPI_VictorSPX(1);
  WPI_VictorSPX right_motor_front = new WPI_VictorSPX(3);

  WPI_VictorSPX shooter = new WPI_VictorSPX(9);
  WPI_VictorSPX belt = new WPI_VictorSPX(10);

  DigitalInput input = new DigitalInput(1);
  

  //

  private final MotorControllerGroup right_Motor_Group = new MotorControllerGroup(right_motor_front, right_motor_back);
  private final MotorControllerGroup left_Motor_Group = new MotorControllerGroup(left_motor_front, left_motor_back);
  DifferentialDrive drivetrain = new DifferentialDrive(left_Motor_Group, right_Motor_Group);


  XboxController xbox = new XboxController(1);
  Joystick joystick = new Joystick(0);

  Timer timer = new Timer();
  
  CANSparkMax intake = new CANSparkMax(7, MotorType.kBrushless);;
  
  enum RobotState { INTAKE, SHOOT };
  RobotState state = RobotState.SHOOT;

  Timer shooter_timer = new Timer();

  /*
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    left_Motor_Group.setInverted(true);
    liftyleft.setNeutralMode(NeutralMode.Brake);
    liftyright.setNeutralMode(NeutralMode.Brake);
    shooterlift.setNeutralMode(NeutralMode.Brake);

    // Prevent integral windup

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
    timer.reset();
    timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // switch (m_autoSelected) {
    //   case kCustomAuto:
    //     break;
    //   case kDefaultAuto:
    //   default:
    //     // Put default auto code here()
    //     break;
    // }

    drivetrain.feedWatchdog();
    // Put custom auto code here

    if (timer.hasElapsed(9.0)) {
      shooterlift.set(0);
    }
    else if (timer.hasElapsed(7.0)) {
      shooterlift.set(0.15);
      state = RobotState.INTAKE;
      shooter_timer.reset();
      shooter_timer.start();
    }
    else if (timer.hasElapsed(4.0)) {
      // don't run
      left_Motor_Group.set(0.0);
      right_Motor_Group.set(0.0);
    }
    else if (timer.hasElapsed(2.0)) {
      // run
      left_Motor_Group.set(0.3);
      right_Motor_Group.set(0.3);
      shooter.set(0);
      belt.set(0);
    }
    else {
      shooter.set(-1);
      belt.set(0.5);
      shooterlift.set(-0.05);
    }
  }



  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    if (!shooter_timer.hasElapsed(2.0)) {
      shooter_timer.reset();
      shooter_timer.start();
    }
  }

  public double evaluatePolynomial(double x) {
      double a = 0.0;
      double b = 1;
      double c = 0.0;

      return a*x + b*java.lang.Math.pow(x, 2) + c*java.lang.Math.pow(x, 3);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
   // drivetrain.arcadeDrive(xbox.getLeftY(), xbox.getLeftX());
    double left_cmd = xbox.getLeftY();
    double right_cmd = xbox.getRightY();

    if (java.lang.Math.abs(left_cmd) < 0.05) {
      left_cmd = 0.0;
    }

    if(java.lang.Math.abs(right_cmd) < 0.05) {
      right_cmd = 0.0;
    }

    double left_cmd_scaled = evaluatePolynomial(java.lang.Math.abs(left_cmd));
    double right_cmd_scaled = evaluatePolynomial(java.lang.Math.abs(right_cmd));

    left_cmd_scaled = java.lang.Math.copySign(left_cmd_scaled, left_cmd);
    right_cmd_scaled = java.lang.Math.copySign(right_cmd_scaled, right_cmd);

    drivetrain.tankDrive(left_cmd_scaled, right_cmd_scaled);
  
    liftyleft.setInverted(false);
    
    if (xbox.getAButton()) {
      liftyleft.set(ControlMode.PercentOutput, .6);    
     } 
    else if (xbox.getXButton()) {
      liftyleft.set(ControlMode.PercentOutput, -.5); 
    } 
    else {
      liftyleft.set(ControlMode.PercentOutput, 0);
    }

    liftyright.setInverted(false);

    if (xbox.getYButton()){
      liftyright.set(ControlMode.PercentOutput,.5);
    }
    else if (xbox.getBButton()) {
      liftyright.set(ControlMode.PercentOutput,-.6); 
    }
    else {
      liftyright.set(ControlMode.PercentOutput, 0);
    }
    
    if (joystick.getRawButton(11) && state == RobotState.SHOOT) {
      // Transtion to intake mode
      state = RobotState.INTAKE;
      shooter_timer.reset();
      shooter_timer.start();
    }
    else if  (joystick.getRawButton(12) && state == RobotState.INTAKE) {
      // Transtion to shoot mode
      state = RobotState.SHOOT;
      shooter_timer.reset();
      shooter_timer.start();
    }

    if (state == RobotState.INTAKE) {
      if (shooter_timer.hasElapsed(2)) {
        shooterlift.set(0.00);
      }
      else {
        shooterlift.set(0.15);
      }

      if (joystick.getRawButton(1)) {
        intake.set(-0.5);
        shooter.set(0.5);
        belt.set(-0.3);
      }
      else {
        intake.set(0);
        shooter.set(0);
        belt.set(0);
      }
    }
    else if (state == RobotState.SHOOT) {
      if (shooter_timer.hasElapsed(4)) {
        shooterlift.set(-0.05);
      }
      else {
        shooterlift.set(-0.35);
      }
      
      if (joystick.getRawButton(1)) {
        shooter.set(-1);
      }
      else {
        shooter.set(0);
      }

      if (joystick.getRawButton(5)){
        belt.set(0.5);
      }
      else if (joystick.getRawButton(3)) {
        belt.set(-0.3);
      }
      else {
        belt.set(0);
      }
    }
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
}
