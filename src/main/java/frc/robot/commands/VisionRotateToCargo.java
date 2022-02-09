// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class VisionRotateToCargo extends CommandBase {
  private Drivetrain m_drivetrain;
  private VisionSubsystem m_visionSubsystem;

  private double m_targetYaw;
  private PhotonPipelineResult m_result;
  private PhotonTrackedTarget m_target;
  private double m_rotationCorrection; 
  private PIDController rotateController = new PIDController(3.0, 0.0, 0.02);

  public VisionRotateToCargo(VisionSubsystem visionSubsystem, Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    m_visionSubsystem = visionSubsystem;

    // TODO: pick red or blue pipeline based on alliance color
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      // m_visionSubsystem.setPipeline(Constants.VisionPipeline.RED_PIPELINE);
    } else {
      // m_visionSubsystem.setPipeline(Constants.VisionPipeline.BLUE_PIPELINE);
    }

    addRequirements(m_drivetrain, m_visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //1 degree tolerance
    rotateController.setTolerance(Math.PI/180);
  }

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
    }
    else {
      // stop rotating if we lose the target
      m_rotationCorrection = 0.0;
    }
    m_drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      0, 0, m_rotationCorrection, m_drivetrain.getGyroscopeRotation()));

    SmartDashboard.putNumber("rotation correction", m_rotationCorrection);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; 
  }
}
