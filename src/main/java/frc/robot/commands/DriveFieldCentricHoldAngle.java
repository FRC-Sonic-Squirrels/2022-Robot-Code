// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveFieldCentricHoldAngle extends CommandBase {
  /** Creates a new DriveFieldCentricHoldAngle. */
  Drivetrain m_drivetrain;
  Rotation2d m_targetYaw = new Rotation2d(0);

  private ProfiledPIDController rotationController = new ProfiledPIDController(1.5, 0.0, 0.0,
      new TrapezoidProfile.Constraints(Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
          Drivetrain.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED));

  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final DoubleSupplier m_rotationSupplier;


  public DriveFieldCentricHoldAngle(Drivetrain drivetrain,
                                    DoubleSupplier translationXSupplier,
                                    DoubleSupplier translationYSupplier,
                                    DoubleSupplier rotationSupplier) {
        
    this.m_translationXSupplier = translationXSupplier;
    this.m_translationYSupplier = translationYSupplier;
    this.m_rotationSupplier = rotationSupplier;



    m_drivetrain = drivetrain;

    rotationController.enableContinuousInput(-Math.PI, Math.PI);
  

    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_targetYaw = m_drivetrain.getRotation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationValue = m_rotationSupplier.getAsDouble() * Constants.DriveFieldCentricConstant.ROTATION_MULTIPLIER;

    if(Math.abs(rotationValue) <= 0.05){
      rotationValue = rotationController.calculate(
                            m_drivetrain.getRotation().getRadians(), m_targetYaw.getRadians());
      if(rotationValue <= 0.5){ 
        rotationValue = 0;
      }
    } else {
      //manual input 
      //if rotating then update the angle we want to hold
      m_targetYaw = m_drivetrain.getRotation();
    }
    

    m_drivetrain.drive(new ChassisSpeeds(
      m_translationXSupplier.getAsDouble(),
      m_translationYSupplier.getAsDouble(), 
      rotationValue));
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
