// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ControllerClimbMaxHeightRumble extends CommandBase {
  /** Creates a new ControllerClimbMaxHeightRumble. */
  XboxController m_controller;
  ElevatorSubsystem m_elevator;
  double m_time = 0;
  double m_count = 0;

  public ControllerClimbMaxHeightRumble(XboxController controller, ElevatorSubsystem elevator) {

    m_controller = controller;
    m_elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Running climb rumble command", true);
    if(m_elevator.m_atMaxheight){
      if(m_count == 0){
        m_controller.setRumble(RumbleType.kRightRumble, 0.5);
        m_controller.setRumble(RumbleType.kLeftRumble, 0.5);

        m_controller.setRumble(RumbleType.kRightRumble, 0.0);
        m_controller.setRumble(RumbleType.kLeftRumble, 0.0);
      }
    } else {
      m_count = 0;
      m_controller.setRumble(RumbleType.kRightRumble, 0.0);
      m_controller.setRumble(RumbleType.kLeftRumble, 0.0);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Running climb rumble command", false);

    m_controller.setRumble(RumbleType.kRightRumble, 0.0);
    m_controller.setRumble(RumbleType.kLeftRumble, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

 
}
