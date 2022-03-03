// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmZeroCommand extends CommandBase {
  private ArmSubsystem arm;
  private double lastPosition = 1.0;  // something really big
  private int repeatCount = 0;

  public ArmZeroCommand(ArmSubsystem arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastPosition = arm.getEncoderValue();
    arm.setArmPercentOutput(-0.45);
    repeatCount = -10;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPosition = arm.getEncoderValue();
    // anything within 0.2/360 is close enough to be unchanged
    if (Math.abs(lastPosition - currentPosition) <= 0.1/360.0) {
      repeatCount++;
    }

    lastPosition = currentPosition;
    System.out.println("Arm Zero Command: " + repeatCount);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setArmPercentOutput(0.0);
    arm.zeroEncoder();
  }

  /**
   * isFinished() - end when we get more than 2 consecutive encoder values that are the same
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
