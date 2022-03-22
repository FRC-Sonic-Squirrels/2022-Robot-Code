// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoSubsystem;

public class CargoRunIndexer extends CommandBase {
  /** Creates a new CargoRunIndexer. */
  CargoSubsystem m_cargo;
  public CargoRunIndexer(CargoSubsystem cargoSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_cargo = cargoSubsystem;
    addRequirements(cargoSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_cargo.setIntakeMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_cargo.setStopMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
