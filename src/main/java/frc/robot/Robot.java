// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.*;

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




  WPI_VictorSPX right_motor_back = new WPI_VictorSPX(4);
  WPI_VictorSPX left_motor_front = new WPI_VictorSPX(2);
  WPI_VictorSPX left_motor_back = new WPI_VictorSPX(1);
  WPI_VictorSPX right_motor_front = new WPI_VictorSPX(3);
  DigitalInput input = new DigitalInput(1);
  

  //

  private final MotorControllerGroup right_Motor_Group = new MotorControllerGroup(right_motor_front, right_motor_back);
  private final MotorControllerGroup left_Motor_Group = new MotorControllerGroup(left_motor_front, left_motor_back);
  DifferentialDrive drivetrain = new DifferentialDrive(right_Motor_Group, left_Motor_Group);


  XboxController xbox = new XboxController(0);

  Timer timer = new Timer();
  
  CANSparkMax sparkmax_motor = new CANSparkMax(7, MotorType.kBrushless);;
  PIDController pid = new PIDController(0.08, 0.4, 0.0);

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

    // Prevent integral windup
    pid.setIntegratorRange(-1.0, 1.0);
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
    switch (m_autoSelected) {
      case kCustomAuto:
        drivetrain.feedWatchdog();
        // Put custom auto code here
        if (timer.hasElapsed(1.0)) {
          // don't run
          left_Motor_Group.set(0.0);
          right_Motor_Group.set(0.0);
        }
        else {
          // run
          left_Motor_Group.set(0.3);
          right_Motor_Group.set(0.3);
        }

        break;
      case kDefaultAuto:
      default:
        // Put default auto code here()
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
   // drivetrain.arcadeDrive(xbox.getLeftY(), xbox.getLeftX());
    drivetrain.tankDrive(xbox.getRightY(), xbox.getLeftY());
    double right_trigger = xbox.getRightTriggerAxis();
    double left_trigger = xbox.getLeftTriggerAxis();
    double set_velocity;

    if (right_trigger > 0.05) {
      set_velocity = right_trigger;
    }
    else if (left_trigger > 0.05) {
      set_velocity = -left_trigger;
    }
    else {
      set_velocity = 0;
    }

    double rots_per_sec = sparkmax_motor.getEncoder().getVelocity() / 1200;
    SmartDashboard.putNumber("encoder value:", sparkmax_motor.getEncoder().getPosition());
    SmartDashboard.putNumber("encoder velocity:", rots_per_sec);

    SmartDashboard.putNumber("velocity set point:", set_velocity * 5);

    double controller_result = pid.calculate(rots_per_sec, set_velocity * 5);
    sparkmax_motor.set(controller_result);
    

    liftyleft.follow(liftyright);
    liftyright.setInverted(false);
    liftyleft.setInverted(InvertType.OpposeMaster);
    if (xbox.getXButton()) {
      liftyright.set(ControlMode.PercentOutput, -.6); 
    } else if (xbox.getBButton()) {
      liftyright.set(ControlMode.PercentOutput, .3); 
    } 
    else {
      liftyright.set(ControlMode.PercentOutput, 0);
    }

    liftyright.setInverted(false);
    liftyleft.setInverted(InvertType.OpposeMaster); 
    if (xbox.getXButton()){
      liftyright.set(ControlMode.PercentOutput,-.6);
    }
    else if (xbox.getBButton()) {
      liftyright.set(ControlMode.PercentOutput,.3); 
    }
    else {
      liftyright.set(ControlMode.PercentOutput, 0);
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
