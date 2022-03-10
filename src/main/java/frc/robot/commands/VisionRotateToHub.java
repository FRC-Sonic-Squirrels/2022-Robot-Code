// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.TestTrajectories;
import frc.robot.subsystems.Drivetrain;

public class VisionRotateToHub extends CommandBase {
  /** Creates a new VisionRotateToHub. */
  private Drivetrain m_drivetrain;
  public VisionRotateToHub(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TestTrajectories.rotateRobot(
      Math.atan((m_drivetrain.getPose().getY()-Constants.HubCentricConstants.HUB_CENTER_POSE2D.getY()) /
      (m_drivetrain.getPose().getX()-Constants.HubCentricConstants.HUB_CENTER_POSE2D.getX()))
      );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
