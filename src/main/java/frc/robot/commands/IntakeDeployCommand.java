// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDeployCommand extends CommandBase {
  /** Creates a new IntakeDeploy. */
  IntakeSubsystem m_intake;
  CargoSubsystem m_cargo;

  double time = 0.0;

  /**
   * Constructor for new IntakeDeployCommand to lower intake, run rollers, and run indexer.
   * @param intakeSubsystem
   * @param cargoSubsystem
   */
  public IntakeDeployCommand(IntakeSubsystem intakeSubsystem, CargoSubsystem cargoSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intakeSubsystem;
    m_cargo = cargoSubsystem;
    addRequirements(m_intake, m_cargo);
  }

  // Called when the command is initially scheduled.
  // will deploy or retract the intake, depending on its current state
  @Override
  public void initialize() {
    m_intake.deployIntake();
    m_intake.setForwardMode();
    //m_intake.setDynamicMode();
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
    m_cargo.setIdleMode(); 
    
    time = 0.0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((m_cargo.cargoInUpperBelts()) && (m_cargo.cargoInLowerBelts())) {
      if (time == 0) {
        time = System.currentTimeMillis();
      } else if (System.currentTimeMillis() - time >= 250) {
        return true;
      }
    }
    else {
      // reset timer if we see a cargo in the indexer
      time = 0;
    }

    // the command will be manually executed and ended by holding a button in teleop
    return false;
  }
}
