// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.ADXL345_I2C.AllAxes;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveScreenCentricCommand extends CommandBase {
  private final Drivetrain m_drivetrainSubsystem;

  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final DoubleSupplier m_rotationSupplier;

  public DriveScreenCentricCommand(Drivetrain drivetrainSubsystem,
                             DoubleSupplier translationXSupplier,
                             DoubleSupplier translationYSupplier,
                             DoubleSupplier rotationSupplier) {
      this.m_drivetrainSubsystem = drivetrainSubsystem;
      this.m_translationXSupplier = translationXSupplier;
      this.m_translationYSupplier = translationYSupplier;
      this.m_rotationSupplier = rotationSupplier;

      addRequirements(drivetrainSubsystem);
  }

  @Override
  public void execute() {
     
    double rotationOutput = m_rotationSupplier.getAsDouble() * Constants.DriveFieldCentricConstant.ROTATION_MULTIPLIER;
      
      if(Math.abs(rotationOutput) <0.05) { rotationOutput = 0.0; }

      int angle = 90;

      if(DriverStation.getAlliance() == Alliance.Red){
        angle = -90;
      }

      m_drivetrainSubsystem.drive(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                      m_translationXSupplier.getAsDouble() * Constants.DriveFieldCentricConstant.TRANSLATION_MULTIPLIER,
                      m_translationYSupplier.getAsDouble() * Constants.DriveFieldCentricConstant.TRANSLATION_MULTIPLIER,
                      rotationOutput,
                      m_drivetrainSubsystem.getRotation().plus(Rotation2d.fromDegrees(angle))
              )
      );
  }

  @Override
  public void end(boolean interrupted) {
      m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}
