// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;

public class HoodZeroAngle extends CommandBase {
  private HoodSubsystem hood;

  public HoodZeroAngle(HoodSubsystem hood) {
    this.hood = hood;
    addRequirements(hood);
  }

  @Override
  public void initialize() {
    hood.setPercentOutput(0.1);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    hood.setPercentOutput(0.0);
    hood.setMinAngle();
  }

  @Override
  public boolean isFinished() {
    return hood.atLowerLimit();
  }

}
