// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoClimbCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class MotionMagicControl extends CommandBase {
  /** Creates a new MotionMagicControl. */
  ElevatorSubsystem m_elevator; 
  double m_target; 
  int m_acel; 
  int m_velo; 
  double m_tolerance; 

  int m_withinThresholdLoops = 0; 
  int kLoopsToSettle = 10; //10*20ms = 200ms = 1/5 of a second 
  
   //TODO: test to see if the time it takes to update the constraints over CAN affects the elevator performance significantly
   //TODO: tune ff and pid in the elevator subsystem 

  public MotionMagicControl(ElevatorSubsystem elevator, double targetHeight, double tolerance,  int acceleration, int cruiseVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.brakeOff();
    m_elevator.setMotionMagicConstraints(m_acel, m_velo);

    m_elevator.setMotionMagicSetPoint(m_target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //has to stay in the tolerance for more than just a split second
    //from https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#mechanism-is-finished-command
    if( Math.abs(m_elevator.getHeightInches() - m_target) < m_tolerance ) {
      m_withinThresholdLoops++;
    } else {
      m_withinThresholdLoops = 0;
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
    return (m_withinThresholdLoops >= kLoopsToSettle);
  }
}
