// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCargoCommand extends CommandBase {
  /** Creates a new ShootTwoCargoCommand. */
  private CargoSubsystem m_cargoSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private IntakeSubsystem m_intakeSubsystem;

  public ShootCargoCommand(CargoSubsystem cargoSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_cargoSubsystem = cargoSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_intakeSubsystem = intakeSubsystem;

    addRequirements(cargoSubsystem, shooterSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.setFlywheelRPM(m_shooterSubsystem.getDesiredRPM());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // wait until flywheel is fully revved
    // once it is, turn on upper cargo belt
    // once upper ball has been released, go back to intake mode

    if (m_shooterSubsystem.isAtDesiredRPM()) {
      m_cargoSubsystem.setBothMode();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setFlywheelRPM(m_shooterSubsystem.getIdleRPM());

    if (m_intakeSubsystem.isDeployed()) {
      m_cargoSubsystem.setIntakeMode();
    }
    else {
      m_cargoSubsystem.setStopMode();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (! m_cargoSubsystem.cargoInUpperBelts()) && (! m_cargoSubsystem.cargoInLowerBelts());
  }
}