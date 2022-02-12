// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmManualControlCommand extends CommandBase {
  Supplier<Double> m_controllerSupplier;
  ArmSubsystem m_arm;
  /** Creates a new ArmManualControlCommand. */
  public ArmManualControlCommand(Supplier<Double> controllerSupplier, ArmSubsystem arm) {
    m_controllerSupplier = controllerSupplier;
    m_arm = arm;
    addRequirements(m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double joystickValue = m_controllerSupplier.get();
    if(joystickValue > 0.05){
      m_arm.setArmPercentOutput(joystickValue);
    }
    if(joystickValue < -0.05){
      m_arm.setArmPercentOutput(joystickValue);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
