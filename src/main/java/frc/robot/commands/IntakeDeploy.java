// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDeploy extends CommandBase {
  /** Creates a new IntakeDeploy. */
  IntakeSubsystem m_intake;
  CargoSubsystem m_cargo;

  public IntakeDeploy(IntakeSubsystem intakeSubsystem, CargoSubsystem cargoSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intakeSubsystem;
    m_cargo = cargoSubsystem;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  // will deploy or retract the intake, depending on its current state
  @Override
  public void initialize() {
    m_intake.deployIntake();
    m_intake.setDynamicMode();
    m_cargo.setIntakeMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.retractIntake();
    m_intake.setStopMode();
    m_cargo.setStopMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
