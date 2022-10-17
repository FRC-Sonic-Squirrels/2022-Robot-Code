// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2930.lib.command;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RumbleControllerOnCondition extends CommandBase {
  /** Creates a new RumbleControllerOnCondition. */
  XboxController m_controller;
  double m_timeToRumble;
  BooleanSupplier m_boolSupplier;
  Boolean m_activated;
  double m_time;

  public RumbleControllerOnCondition(XboxController controller, double timeToRumbleSeconds, BooleanSupplier boolSupplier) {
    m_controller = controller;
    m_timeToRumble = timeToRumbleSeconds;
    m_boolSupplier = boolSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_boolSupplier.getAsBoolean() && !m_activated){
      m_activated = true;
      m_time = System.currentTimeMillis();
      m_controller.setRumble(RumbleType.kRightRumble, 0.6);
      m_controller.setRumble(RumbleType.kRightRumble, 0.6);
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_activated = false;
    m_time = 0;
    m_controller.setRumble(RumbleType.kRightRumble, 0.0);
    m_controller.setRumble(RumbleType.kRightRumble, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!m_activated){
      return false;
    }

    if(System.currentTimeMillis() - m_time >= 1000*m_timeToRumble) {
      return true;
    }

    return false;
  }
}
