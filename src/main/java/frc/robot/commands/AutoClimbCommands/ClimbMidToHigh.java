// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoClimbCommands;

import edu.wpi.first.wpilibj.XboxController;
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
public class ClimbMidToHigh extends SequentialCommandGroup {
  ElevatorSubsystem m_elevator;
  ArmSubsystem m_arm;
  XboxController m_climbController;

  public ClimbMidToHigh(ElevatorSubsystem elevator, ArmSubsystem arm, XboxController climbController) {
    m_elevator = elevator;
    m_arm = arm;
    m_climbController = climbController;
  
    //assumes arms are on mid bar 
    addCommands(
      new ControllerRumbleCommand(m_climbController, 0.2),
      new WaitUntilCommand(() -> confirmButtonPressed()),

      new InstantCommand(() -> m_arm.setArmAngle(Constants.ArmConstants.CLIMBING_FORWARD_ANGLE), m_arm),
      new WaitUntilCommand(() -> m_arm.isAtAngle()),

      new ControllerRumbleCommand(m_climbController, 0.2),
      new WaitUntilCommand(() -> confirmButtonPressed()),
      
      new ElevatorGoToMaxHeight(m_elevator),

      new ControllerRumbleCommand(m_climbController, 0.2),
      new WaitUntilCommand(() -> confirmButtonPressed()),

      new InstantCommand(() -> m_arm.setArmAngle(Constants.ArmConstants.CLIMBING_MIDDLE_ANGLE), m_arm),

      new ControllerRumbleCommand(m_climbController, 0.2),
      new WaitUntilCommand(() -> confirmButtonPressed()),

      new ElevatorGoToSpecificHeight(m_elevator, 10, 1),

      new InstantCommand(() -> m_arm.setArmAngle(Constants.ArmConstants.CLIMBING_BACK_ANGLE), m_arm),
      new WaitUntilCommand(() -> m_arm.isAtAngle()),

      new ElevatorGoToMinHeight(m_elevator),

      new InstantCommand(() -> m_arm.setArmAngle(Constants.ArmConstants.CLIMBING_MIDDLE_ANGLE), m_arm),
      new WaitUntilCommand(() -> m_arm.isAtAngle()),

      new ControllerRumbleCommand(m_climbController, 0.2),
      new WaitUntilCommand(() -> confirmButtonPressed()),

      new ElevatorGoToSpecificHeight(m_elevator, 6, 0.5)
      


    );
  }

  private boolean confirmButtonPressed(){
      return m_climbController.getAButtonPressed();
  }
}
