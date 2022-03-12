// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LimelightRotateToHubAndShoot extends CommandBase {
  private LimelightSubsystem m_limelight;
  private Drivetrain m_drivetrain;
  private CargoSubsystem m_cargoSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private Robot m_robot;
  private PIDController rotateController = new PIDController(3.0, 0.0, 0.02);
  private double m_targetYaw;
  private double m_rotationCorrection;
  private double m_rpm;
  private double m_time;
  private double m_targetAngle;
  /** Creates a new VisionTurnToHub. */
  public LimelightRotateToHubAndShoot(double flywheelRPM, LimelightSubsystem limelight, Drivetrain drivetrain, CargoSubsystem cargoSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, Robot robot) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_rpm = flywheelRPM;
    m_limelight = limelight;
    m_drivetrain = drivetrain;
    m_cargoSubsystem = cargoSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_robot = robot;
    m_time = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("SHOOTING RPM", m_rpm);
    m_shooterSubsystem.setFlywheelRPM(m_rpm);
    m_intakeSubsystem.deployIntake();
  }

  public boolean isAtTargetAngle(){
    return m_drivetrain.getPose().getRotation().getDegrees() + 2 >= m_targetAngle || 
    m_drivetrain.getPose().getRotation().getDegrees() - 2 <= m_targetAngle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_limelight.seesTarget()) {
      m_targetYaw = Math.toRadians(m_limelight.hubRotationDegrees());
      m_targetAngle = m_drivetrain.getPose().getRotation().getDegrees() + m_targetYaw;
      m_rotationCorrection =
          rotateController.calculate(m_drivetrain.getGyroscopeRotation().getRadians(), m_targetYaw)
              * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
      // slow down rotation for testing/safety
      m_rotationCorrection *= 0.3;
      if (m_shooterSubsystem.isAtDesiredRPM() & isAtTargetAngle()) {
        m_cargoSubsystem.setBothMode();
        m_intakeSubsystem.deployIntake();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stop();
    m_cargoSubsystem.setStopMode();
    m_intakeSubsystem.retractIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // the command will automatically stop when both cargo are released in autonomous
    if (m_robot.isAutonomous()) {
      // the command will stop .5 seconds after no cargo is detected, to let the cargo finish shooting
      if ((! m_cargoSubsystem.cargoInUpperBelts()) && (! m_cargoSubsystem.cargoInLowerBelts())) {
        if (m_time == 0) {
          m_time = System.currentTimeMillis();
        }
        if (System.currentTimeMillis() - m_time >= 500) {
          return true;
        }
      }
    }
    // the command will be manually executed and ended by holding a button in teleop
    return false;
  }
}
