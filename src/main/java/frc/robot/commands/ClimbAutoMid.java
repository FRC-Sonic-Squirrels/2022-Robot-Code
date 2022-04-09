// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbAutoMid extends SequentialCommandGroup {
  /** Creates a new ClimbAutoMid. */
  ElevatorSubsystem m_elevator;
  ArmSubsystem m_arm;
  XboxController m_climbController;

  public ClimbAutoMid(ElevatorSubsystem elevator, ArmSubsystem arm, XboxController climbController) {
    m_elevator = elevator;
    m_arm = arm;
    m_climbController = climbController;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> m_arm.setArmAngle(Constants.ArmConstants.CLIMBING_BACK_ANGLE), m_arm),
      new WaitUntilCommand(() -> m_arm.isAtAngle()),

      new InstantCommand(() -> m_elevator.setElevatorHeight(0), m_elevator),
      new WaitUntilCommand(() -> m_elevator.isAtHeight(0)),

      new InstantCommand(() -> m_arm.setArmAngle(Constants.ArmConstants.CLIMBING_MIDDLE_ANGLE), m_arm),
      new WaitUntilCommand(() -> m_arm.isAtAngle()),

      new InstantCommand(() -> m_elevator.setElevatorHeight(6), m_elevator),
      new WaitUntilCommand(() -> m_elevator.isAtHeight(6)),

      new ControllerRumbleCommand(m_climbController, 0.2)
    );
  }
}
