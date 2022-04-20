// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoClimbCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorGoToSpecificHeight extends CommandBase {
  /** Creates a new ElevatorGoToSpecificHeight. */
  ElevatorSubsystem m_elevator;
  double m_targetHeight;
  double m_strength;


  //Use this as a way to bypass the low pid on elevator? we dont need to be accurate 
  //unless we need to be at max or min and those commands are accurate


  public ElevatorGoToSpecificHeight(ElevatorSubsystem elevator, double targetHeight, double strength) {
    m_elevator = elevator;
    m_targetHeight = targetHeight;
    m_strength = strength;

    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_elevator.getHeightInches() < m_targetHeight){
       m_elevator.setWinchPercentOutput(-m_strength);
    } else if(m_elevator.getHeightInches() > m_targetHeight){
      m_elevator.setWinchPercentOutput(m_strength);
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
    return m_elevator.isAtHeight(m_targetHeight);
  }
}
