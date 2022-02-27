// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Instant;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.RunnableFuture;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ClimbManualCommand extends CommandBase {
  /** Creates a new ClimbManualCommand. */
  ArmSubsystem m_arm;
  ElevatorSubsystem m_elevator;
  XboxController m_controller;

  public ClimbManualCommand(ArmSubsystem arm, ElevatorSubsystem elevator, XboxController controller) {
    m_arm = arm;
    m_elevator = elevator;
    m_controller = controller;

    //if left bumper pressed again then end command 
    withInterrupt(() -> m_controller.getLeftBumperPressed());
    addRequirements(arm, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rumbleSequenceCommand().schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO: check the arm multiplier for changes 
    double armJoyStickValue = m_controller.getRightY();
    if(Math.abs(armJoyStickValue) > 0.1){
      m_arm.setArmPercentOutput(armJoyStickValue * 0.3);
    }

    double elevatorJoyStickValue = m_controller.getLeftY();
    if(Math.abs(elevatorJoyStickValue) > 0.1) {
      m_elevator.brakeOff();
      m_elevator.setWinchPercentOutput(elevatorJoyStickValue * Constants.ElevatorConstants.elevatorSpeedMultiplier);
    } else {
      m_elevator.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rumbleSequenceCommand().schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  //rumble wait 1 second then end rumble 
  private Command rumbleSequenceCommand(){
    return new SequentialCommandGroup(
      new InstantCommand(() -> m_controller.setRumble(RumbleType.kLeftRumble, 1)),
      new InstantCommand(() -> m_controller.setRumble(RumbleType.kRightRumble, 1)),
      new WaitCommand(1),
      new InstantCommand(() -> m_controller.setRumble(RumbleType.kLeftRumble, 0)),
      new InstantCommand(() -> m_controller.setRumble(RumbleType.kRightRumble, 0))
    );
  }
}
