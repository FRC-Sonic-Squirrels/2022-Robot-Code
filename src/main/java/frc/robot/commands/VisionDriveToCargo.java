// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.SwerveTrajectoryFollowCommandFactory;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class VisionDriveToCargo extends CommandBase {
  private VisionSubsystem m_visionSubsystem;
  private Drivetrain m_drivetrain;

  private PhotonPipelineResult m_result;
  private PhotonTrackedTarget m_target;
  private double m_targetYaw;
  private double m_rotationCorrection;
  private double m_forwardVelocity;

  PIDController rotateController = new PIDController(3.0, 0.0, 0.02);
  //max for the trajectory set to low for safety 
  private double m_maxVelocity = 0.5;
  private double m_maxAcceleration = 0.25;

  public VisionDriveToCargo(VisionSubsystem visionSubsystem, Drivetrain drivetrain) {
    m_visionSubsystem = visionSubsystem;
    m_drivetrain = drivetrain;

    addRequirements(visionSubsystem, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_result = m_visionSubsystem.getResult();
   
    if(m_result.hasTargets()){
      m_target = m_result.getBestTarget();
      //negate because of how robot rotates 
      m_targetYaw = -Math.toRadians(m_target.getYaw());
      
      m_rotationCorrection = rotateController.calculate(0, m_targetYaw) 
      * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
      //slow down rotation for testing/safety 
      m_rotationCorrection *= 0.3;

      m_forwardVelocity = Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * 0.05;
    } else{
      // stop rotating if we lose the target
      m_rotationCorrection = 0.0;
      m_forwardVelocity = 0.0;
    }
    m_drivetrain.drive(new ChassisSpeeds(
      m_forwardVelocity, 0, m_rotationCorrection));

    SmartDashboard.putNumber("rotation correction", m_rotationCorrection);
   
    // if(m_result.hasTargets()){
    //   m_transformationToCargo = m_result.getBestTarget().getCameraToTarget();
    //   Pose2d pose = new Pose2d(
    //     m_transformationToCargo.getTranslation(),
    //     m_transformationToCargo.getRotation());

    //   TrajectoryConfig config = new TrajectoryConfig(m_maxVelocity, m_maxAcceleration)
    //       .setKinematics(m_drivetrain.kinematics());

    //   SwerveDriveKinematicsConstraint swerveConstraint = new SwerveDriveKinematicsConstraint(
    //       m_drivetrain.kinematics(), Drivetrain.MAX_VELOCITY_METERS_PER_SECOND);
    //   config.addConstraint(swerveConstraint);

    //   var trajectory = TrajectoryGenerator.generateTrajectory(
    //     new Pose2d(), 
    //     List.of(), 
    //     pose, 
    //     config);

    //   var command = SwerveTrajectoryFollowCommandFactory.SwerveControllerCommand(trajectory, m_drivetrain, true);  
    //   //will this work? the new command will be different and interrupt the previous command because it uses the same subsystems
    //   //in theory it should interrupt the pervious command and schedule a new one
    //   //But how do we interrupt the last running command when we let go of the button? 
    //   //Possible fix for that in the end method
    //   CommandScheduler.getInstance().schedule(command);
    // }

  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //gets the command using the drivetrain this should be the trajectory command 
    var command = CommandScheduler.getInstance().requiring(m_drivetrain);
    //cancles any command using the drive train probably the trajectory command at this time 
    //so drivetrain doesnt have any command using it at this moment thus it will return to its 
    //defualt driving command  
    if(command!=null){
      command.end(true);
    }
    m_drivetrain.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
