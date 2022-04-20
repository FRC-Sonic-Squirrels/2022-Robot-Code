// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ElevatorZeroHeight;
import frc.robot.commands.HoodZeroAngle;

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

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer(this);

    // clear sticky faults
    revPDH.clearStickyFaults();
    revPDH.resetTotalEnergy();

    revPDH.setSwitchableChannel(true);

    // We don't use this
    LiveWindow.disableAllTelemetry();

    // log NetworkTables data
    DataLogManager.start();

    if (isReal()) {
      // Creates UsbCamera and sets resolution
      camera = CameraServer.startAutomaticCapture();
      camera.setResolution(160, 120);
      camera.setFPS(30);
    }

    //m_robotContainer.updateManualShooterSettings();
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

    if (!isAutonomous()) {
      m_robotContainer.updateManualShooterSettings();
    }
    
    // SmartDashboard.putNumber("Joystick_Values jLeftY", m_robotContainer.m_controller.getLeftY());
    // SmartDashboard.putNumber("Joystick_Values jLeftX", m_robotContainer.m_controller.getLeftX());
    // SmartDashboard.putNumber("Joystick_Values jRightY", m_robotContainer.m_controller.getRightY());
    // SmartDashboard.putNumber("Joystick_Values jRightX", m_robotContainer.m_controller.getRightX());

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.m_shooter.stop();
    m_robotContainer.m_cargo.coastMode();
    m_robotContainer.m_hood.setMinAngle();
    m_robotContainer.m_shooter.stop();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.chooser.getSelected();
 
    new HoodZeroAngle(m_robotContainer.m_hood).schedule(true);
    new ElevatorZeroHeight(m_robotContainer.m_elevator).schedule(true);

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

    new HoodZeroAngle(m_robotContainer.m_hood).schedule(true);
    new ElevatorZeroHeight(m_robotContainer.m_elevator).schedule(true);

    //m_robotContainer.climbRumbleCommand.schedule(false);

    // Pose2d start = new Pose2d(8.23 - Units.inchesToMeters(138), 4.11, new Rotation2d(Math.PI));

    // //TODO: remove before auto
    // m_robotContainer.drivetrain.setPose(start, m_robotContainer.drivetrain.getIMURotation());

    //if testing and just using teleop we reset pose and rotation to 0, auton will correct this 
    //for its own use case and continue working after u switch to teleop
    if(!m_robotContainer.drivetrain.isOdometrySet()){
      m_robotContainer.drivetrain.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0)));
    }

 

    // m_robotContainer.drivetrain.setDefaultCommand(new DriveFieldCentricCommand(
    //   m_robotContainer.drivetrain, 
    //   () -> -RobotContainer.modifyAxis(m_robotContainer.m_controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
    //   () -> -RobotContainer.modifyAxis(m_robotContainer.m_controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 
    //   () -> -RobotContainer.modifyAxis(m_robotContainer.m_controller.getRightX() * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)));

    // m_robotContainer.m_elevator.setDefaultCommand(new ElevatorControlCommand(m_robotContainer.m_elevator, m_robotContainer.m_climbController,
    //     Constants.ElevatorConstants.elevatorSpeedMultiplier));

    // m_robotContainer.m_arm.setDefaultCommand(new ArmManualControlCommand(m_robotContainer.m_arm, m_robotContainer.m_climbController, 0.3));

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //m_robotContainer.updateManualShooterSettings();
    //SmartDashboard.putBoolean("Climb Rumble Command scheduled", m_robotContainer.climbRumbleCommand.isScheduled()); 
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

}
