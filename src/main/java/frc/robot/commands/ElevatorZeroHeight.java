// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorZeroHeight extends CommandBase {
  private ElevatorSubsystem m_elevator;

  /** Creates a new ElevatorZeroHeight. */
  public ElevatorZeroHeight(ElevatorSubsystem elevator) {
    m_elevator = elevator;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setWinchPercentOutput(0.2);
    m_elevator.brakeOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.stop();
    m_elevator.brakeOn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.atLowerLimit();
  }
}
