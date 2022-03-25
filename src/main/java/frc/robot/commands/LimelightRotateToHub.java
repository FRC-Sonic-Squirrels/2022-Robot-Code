// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightRotateToHub extends CommandBase {
  /** Creates a new LimelightRotateToHub. */
  private LimelightSubsystem m_limelight;
  private Drivetrain m_drivetrain;
  

  private ProfiledPIDController rotationalController = new ProfiledPIDController(3.0, 0.0, 0.02,
      new TrapezoidProfile.Constraints(
          Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 5,
          Drivetrain.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED * 0.4));

  private double m_targetYaw;
  private double m_targetAngle;
  private double m_rotationCorrection;

  public LimelightRotateToHub(LimelightSubsystem limelight, Drivetrain drivetrain) {
    m_limelight = limelight;
    m_drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_limelight.seesTarget()) {
      m_targetYaw = Math.toRadians(m_limelight.hubRotationDegrees());
      m_targetAngle = m_drivetrain.getPose().getRotation().getRadians() + m_targetYaw;
      m_rotationCorrection =
          rotationalController.calculate(m_drivetrain.getRotation().getRadians(), m_targetAngle);
        // slow down rotation for testing/safety
        m_rotationCorrection *= 0.3;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
