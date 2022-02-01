// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.SwerveTrajectoryFollowCommandFactory;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class VisionDriveToCargo extends CommandBase {
  private VisionSubsystem m_visionSubsystem;
  private Drivetrain m_drivetrain;

  private Transform2d m_transformationToCargo;

  //max for the trajectory set to low for safety 
  private double m_maxVelocity = 1.0;
  private double m_maxAcceleration = 0.75;

  public VisionDriveToCargo(VisionSubsystem visionSubsystem, Drivetrain drivetrain) {
    m_visionSubsystem = visionSubsystem;
    m_drivetrain = drivetrain;

    m_transformationToCargo = m_visionSubsystem.getPoseToCargo();

    addRequirements(visionSubsystem, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d pose = new Pose2d(
      m_transformationToCargo.getTranslation(),
      m_transformationToCargo.getRotation());

    TrajectoryConfig config = new TrajectoryConfig(m_maxVelocity, m_maxAcceleration)
        .setKinematics(m_drivetrain.kinematics());

    SwerveDriveKinematicsConstraint swerveConstraint = new SwerveDriveKinematicsConstraint(
        m_drivetrain.kinematics(), Drivetrain.MAX_VELOCITY_METERS_PER_SECOND);
    config.addConstraint(swerveConstraint);

    var trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(), 
      List.of(), 
      pose, 
      config);

    var command = SwerveTrajectoryFollowCommandFactory.SwerveControllerCommand(trajectory, m_drivetrain, true);
    CommandScheduler.getInstance().schedule(command);
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
