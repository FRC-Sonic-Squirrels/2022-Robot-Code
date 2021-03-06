// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

public class DriveRobotCentricCommand extends CommandBase {
  /** Creates a new RobotCentricDriving. */
    private final Drivetrain m_Drivetrain;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

  

    public DriveRobotCentricCommand(Drivetrain Drivetrain,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier) {

      this.m_Drivetrain = Drivetrain;
      this.m_translationXSupplier = translationXSupplier;
      this.m_translationYSupplier = translationYSupplier;
      this.m_rotationSupplier = rotationSupplier;

      addRequirements(Drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Drivetrain.drive(
      new ChassisSpeeds(
            m_translationXSupplier.getAsDouble() * Constants.DriveFieldCentricConstant.TRANSLATION_MULTIPLIER,
            m_translationYSupplier.getAsDouble() * Constants.DriveFieldCentricConstant.TRANSLATION_MULTIPLIER,
            m_rotationSupplier.getAsDouble() * Constants.DriveFieldCentricConstant.ROTATION_MULTIPLIER)  
        );
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
