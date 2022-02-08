// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class CargoReverseCommand extends CommandBase {
  /** Creates a new CargoReverseCommand. */
  CargoSubsystem m_cargo;
  IntakeSubsystem m_intake;
  public CargoReverseCommand(CargoSubsystem cargo, IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_cargo = cargo;
    m_intake = intake;
    addRequirements(cargo, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_cargo.setReverseMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_intake.isDeployed()) {
      m_cargo.setIntakeMode();
    } else {
      m_cargo.setStopMode();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
