// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoClimbCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

      new ArmSetAngle(m_arm, Constants.ArmConstants.CLIMBING_FORWARD_ANGLE)
      .withTimeout(0.1),

      new ControllerRumbleCommand(m_climbController, 0.2),
      new WaitUntilCommand(() -> confirmButtonPressed()),
      
      new ElevatorGoToMaxHeight(m_elevator,1.0),

      new ControllerRumbleCommand(m_climbController, 0.2),
      new WaitUntilCommand(() -> confirmButtonPressed()),

      //elevator hits high bar 
      new ArmSetAngle(m_arm, Constants.ArmConstants.CLIMBING_MIDDLE_ANGLE)
      .withTimeout(0.1),


      new ControllerRumbleCommand(m_climbController, 0.2),
      new WaitUntilCommand(() -> confirmButtonPressed()),

      //latch on to high and mid 
     // new ElevatorGoToSpecificHeight(m_elevator, 23, 1, 0.4),

      new ControllerRumbleCommand(m_climbController, 0.2),
      new WaitUntilCommand(() -> confirmButtonPressed()),

      //gentle arm lift off 
    new SequentialCommandGroup(
      new ElevatorGoToSpecificHeight(m_elevator, 15, 0.5, 0.4),
      new ArmSetAngle(m_arm, Constants.ArmConstants.CLIMBING_BACK_ANGLE)
        .withTimeout(0.1)
    ),
    
     // new ControllerRumbleCommand(m_climbController, 0.2),
      // new WaitUntilCommand(() -> confirmButtonPressed()),

      new ControllerRumbleCommand(m_climbController, 0.2),
      new WaitUntilCommand(() -> confirmButtonPressed()),

      new ElevatorGoToSpecificHeight(m_elevator, 0, 0.5, 0.4),

      
      new ControllerRumbleCommand(m_climbController, 0.2),
      new WaitUntilCommand(() -> confirmButtonPressed()),

      new ArmSetAngle(m_arm, Constants.ArmConstants.CLIMBING_MIDDLE_ANGLE)
      .withTimeout(0.1),

      new ControllerRumbleCommand(m_climbController, 0.2),
      new WaitUntilCommand(() -> confirmButtonPressed()),

      new ElevatorGoToSpecificHeight(m_elevator, 5, 0.6, 0.4)
    );
  }

  private boolean confirmButtonPressed(){
      return m_climbController.getAButtonPressed();
  }
}
