// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorControlCommand extends CommandBase {
  ElevatorSubsystem elevator;
  XboxController controller;
  double gain = 1.0;

  public ElevatorControlCommand(ElevatorSubsystem elevator, XboxController controller, double gain) {
    this.elevator = elevator;
    this.controller = controller;
    this.gain = gain;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double elevatorJoyStickValue = controller.getLeftY();

    if (controller.getYButton()) {
      elevator.setElevatorHeight(19.0);
    } else if (controller.getBButton()) {
      elevator.setElevatorHeight(6.0);
    } else if (controller.getAButton()) {
      elevator.setElevatorHeight(1.0);
    } else if (controller.getXButton()) {
      elevator.setElevatorHeight(11.0);
    }
    else if(elevatorJoyStickValue > 0.1) {
      elevator.setWinchPercentOutput(elevatorJoyStickValue * gain);
    }

  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
