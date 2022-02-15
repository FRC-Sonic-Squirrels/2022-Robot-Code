// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorControlCommand extends CommandBase {
  /** Creates a new ElevatorDeploy. */
  Supplier<Double> m_controllerSupplier;
  ElevatorSubsystem m_elevator;

  public ElevatorControlCommand(Supplier<Double> controllerSupplier, ElevatorSubsystem elevator) {
    m_elevator = elevator;
    m_controllerSupplier = controllerSupplier;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(Math.abs(m_controllerSupplier.get()) >= 0.1){
      m_elevator.brakeOff();
      m_elevator.setWinchPercentOutput(m_controllerSupplier.get()*ElevatorConstants.elevatorSpeedMultiplier);
    } else {
      m_elevator.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
