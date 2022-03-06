// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CargoSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public final PowerDistribution revPDH = new PowerDistribution();

  private UsbCamera camera;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {

    SmartDashboard.putNumber("SHOOTING RPM", 2000);
    
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer(this);

    // clear sticky faults
    revPDH.clearStickyFaults();
    revPDH.resetTotalEnergy();

    // We don't use this
    LiveWindow.disableAllTelemetry();

    

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Joystick_Values jLeftY", m_robotContainer.m_controller.getLeftY());
    SmartDashboard.putNumber("Joystick_Values jLeftX", m_robotContainer.m_controller.getLeftX());
    SmartDashboard.putNumber("Joystick_Values jRightY", m_robotContainer.m_controller.getRightY());
    SmartDashboard.putNumber("Joystick_Values jRightX", m_robotContainer.m_controller.getRightX());

    //SmartDashboard.putNumber("PDH Total Power", revPDH.getTotalPower());
    // FIXME: getTotalCurrent() throws errors
    //SmartDashboard.putNumber("PDH Total Current", revPDH.getTotalCurrent());
    //SmartDashboard.putNumber("PDH Total Energy", revPDH.getTotalEnergy());

    

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.m_cargoSubsystem.coastMode();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.chooser.getSelected();
    Pose2d startPos = Constants.StartPoseConstants.BLUE_MID_TOP;

    m_robotContainer.drivetrain.setGyroscopeHeadingDegrees(startPos.getRotation().getDegrees());
    m_robotContainer.drivetrain.setPose(startPos, m_robotContainer.drivetrain.getGyroscopeRotation());

    m_autonomousCommand = new InstantCommand(
      () ->m_robotContainer.drivetrain.drive(new ChassisSpeeds()), m_robotContainer.drivetrain)
        .perpetually();


    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

}
