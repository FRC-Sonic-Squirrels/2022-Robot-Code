// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class VisionRotateToCargo extends CommandBase {
  private Drivetrain m_drivetrain;
  private VisionSubsystem m_visionSubsystem;

  private double m_targetYaw;
  private PhotonTrackedTarget m_target;
  private double m_rotationCorrection; 
  private PIDController rotatController = new PIDController(3.0, 0.0, 0.02);

  public VisionRotateToCargo(VisionSubsystem visionSubsystem, Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    m_visionSubsystem = visionSubsystem;

    addRequirements(m_drivetrain, m_visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //1 degree tolorence
    rotatController.setTolerance(Math.PI/180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_target = m_visionSubsystem.getTarget();
    m_targetYaw = m_target.getYaw();
    //negate because of how robot rotates 
    m_targetYaw = -Math.toRadians(m_targetYaw);

    m_rotationCorrection = rotatController.calculate(0, m_targetYaw) 
    * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    //slow down rotation for testing/safety 
    m_rotationCorrection *= 0.5;
    
    m_drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      0, 0, m_rotationCorrection, m_drivetrain.getGyroscopeRotation()));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  false;
  
    
  }
}
