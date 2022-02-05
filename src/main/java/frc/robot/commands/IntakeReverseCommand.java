// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeReverseCommand extends CommandBase {
  IntakeSubsystem m_intake;
  /** Creates a new IntakeReverseCommand. */
  public IntakeReverseCommand(IntakeSubsystem intake) {
    m_intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.deployIntake();
    m_intake.setReverseMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  //TODO: have this command deploy, set intake to reverse, then stop
  @Override
  public void end(boolean interrupted) {
    m_intake.setStopMode();
    m_intake.retractIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
