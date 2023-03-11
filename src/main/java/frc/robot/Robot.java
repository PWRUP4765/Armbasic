// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
/* Talon SRX 6 is claw Talon SRX 5 is pitch Talon SRX 9 is I/O */
  //private final TalonSRX m_motordrive_pitch = new TalonSRX(5);
  //private final TalonSRX m_motordrive_io = new TalonSRX(9);
  //private final DutyCycleEncoder m_encoder_pitch = new DutyCycleEncoder (0); // pitch
  //private final DutyCycleEncoder m_encoder_io = new DutyCycleEncoder (5); // IO
  
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();

  private final XboxController m_controller = new XboxController(0);
  private final Timer m_timer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
   // m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX());
   double output_pitch = m_armSubsystem.getPosPitch();
   double output_reach = m_armSubsystem.getPosReach();
   double control_pitch = m_controller.getRawAxis(1); // Left Stick Vertical
   double control_reach = m_controller.getRawAxis(3); // Right Stick Horizontal

   SmartDashboard.putNumber("encoder_pitch", output_pitch);
   SmartDashboard.putNumber("encoder_reach", output_reach);
   SmartDashboard.putNumber("controller_left_1", control_pitch);
   SmartDashboard.putNumber("controller_right_2", control_reach);

  m_armSubsystem.movePitch(control_pitch);
  m_armSubsystem.moveReach(control_reach);

   //m_motordrive_pitch.set(ControlMode.PercentOutput, control_pitch*0.5);
   //m_motordrive_io.set(ControlMode.PercentOutput, control_reach*0.5);
}

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
