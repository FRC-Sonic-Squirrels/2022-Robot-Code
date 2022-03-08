// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeReverseCommand extends CommandBase {
  IntakeSubsystem m_intake;
  CargoSubsystem m_cargo;

  public IntakeReverseCommand(IntakeSubsystem intake, CargoSubsystem cargo) {
    m_intake = intake;
    m_cargo = cargo;
    addRequirements(intake, cargo);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.deployIntake();
    m_intake.setReverseMode();
    m_cargo.setReverseMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setStopMode();
    m_intake.retractIntake();
    m_cargo.setStopMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
