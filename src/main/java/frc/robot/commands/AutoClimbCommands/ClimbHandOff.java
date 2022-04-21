// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoClimbCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.ControllerRumbleCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbHandOff extends SequentialCommandGroup {
  ElevatorSubsystem m_elevator;
  ArmSubsystem m_arm;
  XboxController m_climbController;

  public ClimbHandOff(ElevatorSubsystem elevator, ArmSubsystem arm, XboxController climbController, double endElevatorHeight) {
    m_elevator = elevator;
    m_arm = arm;
    m_climbController = climbController;
    addCommands(
      new ControllerRumbleCommand(m_climbController, 0.2),
      new WaitUntilCommand(() -> confirmButtonPressed()),

      new ArmSetAngle(m_arm, Constants.ArmConstants.CLIMBING_BACK_ANGLE),

      new ControllerRumbleCommand(m_climbController, 0.2),
      new WaitUntilCommand(() -> confirmButtonPressed()),

      new ElevatorGoToMinHeight(m_elevator),

      new ControllerRumbleCommand(m_climbController, 0.2),
      new WaitUntilCommand(() -> confirmButtonPressed()),

      new ArmSetAngle(m_arm, Constants.ArmConstants.CLIMBING_MIDDLE_ANGLE),
      
      new ControllerRumbleCommand(m_climbController, 0.2),
      new WaitUntilCommand(() -> confirmButtonPressed()),

      // TODO: guess that 22 inches gets us under the next bar
      new ElevatorGoToSpecificHeight(m_elevator, endElevatorHeight, 1.0,0.4)

    );
  }

  private boolean confirmButtonPressed() {
    return m_climbController.getAButtonPressed();
  }
}
