// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * This class is for generating trajectories. It includes several trajectories for calibrating
 * trajectory following.
 */
public class TestTrajectories {

  private boolean isSwerve = true;

  // These are very tame velocity and acceleration values. Relatively save for testing.
  private double maxVelocity = 1.0;
  private double maxAcceleration = 0.75;

  private DrivetrainSubsystem drivetrainSubsystem = null;

  /**
   * Constructor for Test Trajectory factory.
   * 
   * @param maxVelocity
   * @param maxAcceleration
   * @param drivetrainSubsystem
   * @param isSwerve
   */
  public TestTrajectories(double maxVelocity, double maxAcceleration,
      DrivetrainSubsystem drivetrainSubsystem, boolean isSwerve) {
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.isSwerve = isSwerve;
  }

  public TrajectoryConfig getTrajectoryConfig() {
    TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(drivetrainSubsystem.kinematics());

    if (isSwerve) {
      // Limits the velocity of the robot around turns such that no wheel of a swerve-drive robot
      // goes over a specified maximum velocity.
      SwerveDriveKinematicsConstraint swerveConstraint = new SwerveDriveKinematicsConstraint(
          drivetrainSubsystem.kinematics(), DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND);
      config.addConstraint(swerveConstraint);
    }
    return config;
  }

  /**
   * Straight trajectory
   * 
   * Return a trajectory that drives straight for a given distance in meters.
   * 
   * @param distanceInMeters
   * @return trajectory
   */
  public Trajectory straightForward(double distanceInMeters) {

    // setReversed(true) if we are traveling backwards

    return TrajectoryGenerator.generateTrajectory(new Pose2d(0.0, 0.0, new Rotation2d(0)),
        List.of(), new Pose2d(distanceInMeters, 0.0, new Rotation2d(0)),
        getTrajectoryConfig().setReversed(distanceInMeters < 0.0));

  }

  /**
   * Sideways trajectory
   * 
   * Return a trajectory that drives sideways for a given distance in meters.
   * 
   * This will only work for holonomic drivetrains, like swerve.
   * 
   * @param distanceInMeters
   * @return trajectory
   */
  // TODO: this is a terrible function name. fix this.
  public Trajectory straightSideways(double distanceInMeters) {

    if( isSwerve ) {
      return TrajectoryGenerator.generateTrajectory(new Pose2d(0.0, 0.0, new Rotation2d(0)),
          List.of(), new Pose2d(0.0, distanceInMeters, new Rotation2d(0)),
          getTrajectoryConfig());
    }
    else {
      // instead we return a do nothing trajectory (0,0) -> (0,0)
      System.out.println("WARNING: non holonomic drive can't drive sideways!");
      return TrajectoryGenerator.generateTrajectory(new Pose2d(0.0, 0.0, new Rotation2d(0)),
      List.of(), new Pose2d(0.0, 0.0, new Rotation2d(0)),
      getTrajectoryConfig());
    }

  }

  /**
   * 
   * Return an trajectory that drives forward and to the left/right for a given distances in meters.
   * 
   * @param forwardInMeters
   * @param leftInMeters
   * @return trajectory
   */
  public Trajectory simpleCurve(double forwardInMeters, double leftInMeters) {

    double rotation = Math.PI / 2;

    if (leftInMeters == 0) {
      rotation = 0;
    } else if (leftInMeters < 0) {
      // turning to the right ("the other left")
      rotation = -1.0 * Math.PI / 2.0;
    }

    return TrajectoryGenerator.generateTrajectory(new Pose2d(0.0, 0.0, new Rotation2d(0)),
        List.of(), new Pose2d(forwardInMeters, leftInMeters, new Rotation2d(rotation)),
        getTrajectoryConfig());
  }

  /**
   * 
   * Return an trajectory that drives a figure eight pattern. Define the radius of curves in meters.
   * 
   * @param radiusInMeters
   * @return trajectory
   */
  public Trajectory figureEight(double radius) {

    return TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(radius, -radius), new Translation2d(2.0 * radius, -2.0 * radius),
            new Translation2d(3.0 * radius, -radius), new Translation2d(2.0 * radius, 0.0),
            new Translation2d(radius, -radius), new Translation2d(0.0, -2.0 * radius),
            new Translation2d(-radius, -radius)),
        new Pose2d(0.0, 0.0, new Rotation2d(0)), getTrajectoryConfig());
  }

}
