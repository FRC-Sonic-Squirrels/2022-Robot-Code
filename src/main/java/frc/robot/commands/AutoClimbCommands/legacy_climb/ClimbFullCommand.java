// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoClimbCommands.legacy_climb;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbFullCommand extends SequentialCommandGroup {
  /** Creates a new ClimbFullCommand. */
  ElevatorSubsystem m_elevator;
  ArmSubsystem m_arm;
  XboxController m_climbController;

  public ClimbFullCommand(ElevatorSubsystem elevator, ArmSubsystem arm, XboxController climbController) {
    m_elevator = elevator;
    m_arm = arm;
    m_climbController = climbController;

    addCommands(
      new ClimbMidAuto(m_elevator, m_arm, m_climbController),

      new ClimbMidToHigh(m_elevator, m_arm, m_climbController),

      new ClimbHighToTraverse(m_elevator, m_arm, climbController)
    );
  }
}
