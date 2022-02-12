// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmAutoControlCommand extends CommandBase {
  Supplier<Double> m_POVSupplier;
  ArmSubsystem m_arm;
  /** Creates a new ArmAutoControlCommand. */
  public ArmAutoControlCommand(Supplier<Double> POVSupplier, ArmSubsystem arm) {
    m_POVSupplier = POVSupplier;
    m_arm = arm;
    addRequirements(m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double dPadValue = m_POVSupplier.get();
    if(dPadValue == 270.0){
      m_arm.setArmToSpecificRotation(Constants.ArmConstants.m_minEncoderValue); //all the way back
    }
    else if(dPadValue == 90.0){
      m_arm.setArmToSpecificRotation(Constants.ArmConstants.m_maxEncoderValue); //all the way front
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
