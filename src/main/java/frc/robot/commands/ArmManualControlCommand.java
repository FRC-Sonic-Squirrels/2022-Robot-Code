// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmManualControlCommand extends CommandBase {
  XboxController controller;
  ArmSubsystem arm;
  boolean holding = false;
  double gain = 0.3;

  public ArmManualControlCommand(ArmSubsystem arm, XboxController controller) {
    this.controller = controller;
    this.arm = arm;
    addRequirements(arm);
  }

  /*
  * ArmManualControlCommand - control the arm with the xbox controller.
  * 
  * @param arm - the arm subsystem
  * @param controller - the xbox controller
  * @param gain - trigger multiplier (0-1.0), 1.0 is full power
  */
  public ArmManualControlCommand(ArmSubsystem arm, XboxController controller, double gain) {
    this.controller = controller;
    this.arm = arm;
    this.gain = gain;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Arm Command POV", -1);
  }

  @Override
  public void execute() {
    double leftTrigger = controller.getLeftTriggerAxis();
    double rightTrigger = controller.getRightTriggerAxis();
    double pov = controller.getPOV();

    if (pov == 0) {
      arm.setArmAngle(Constants.ArmConstants.CLIMBING_MIDDLE_ANGLE);
      holding = true;
    } else if (pov == 90) {
      arm.setArmAngle(Constants.ArmConstants.CLIMBING_FORWARD_ANGLE);
      holding = true;
    } else if (pov == 180) {
      arm.setArmAngle(Constants.ArmConstants.CLIMBING_NEXT_BAR_ANGLE);
      holding = true;
    } else if (pov == 270) {
      arm.setArmAngle(Constants.ArmConstants.CLIMBING_BACK_ANGLE);
      holding = true;
    }
    else if(leftTrigger > 0.1){
      arm.setArmPercentOutput(- leftTrigger * gain);
      holding = false;
    }
    else if(rightTrigger > 0.1){
      arm.setArmPercentOutput(rightTrigger * gain);
      holding = false;
    }
    else if (!holding){
      arm.hold();
      holding = true;
    }

    SmartDashboard.putBoolean("Arm Command Holding", holding);

    if (pov != -1) {
      SmartDashboard.putNumber("Arm Command POV", pov);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.hold();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
