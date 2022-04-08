// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ControllerRumbleCommand extends CommandBase {
  /** Creates a new ControllerRumbleCommand. */
  XboxController m_controller;
  double m_timeToRumbleSeconds;
  double m_time;


  public ControllerRumbleCommand(XboxController controller, double timeToRumbleSeconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_controller = controller;
    m_timeToRumbleSeconds = timeToRumbleSeconds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller.setRumble(RumbleType.kRightRumble, 0.1);
    m_controller.setRumble(RumbleType.kLeftRumble, 0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_controller.setRumble(RumbleType.kRightRumble, 0.0);
    m_controller.setRumble(RumbleType.kLeftRumble, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_time == 0) {
      m_time = System.currentTimeMillis();
    } else if (System.currentTimeMillis() - m_time >= 1000*m_timeToRumbleSeconds) {
      return true;
    }

    return false;
  }
}
