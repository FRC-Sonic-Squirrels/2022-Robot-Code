// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ClimbManualCommand extends CommandBase {
  ArmSubsystem m_arm;
  ElevatorSubsystem m_elevator;
  XboxController m_controller;
  boolean m_isUnlocked = false;

  public ClimbManualCommand(ArmSubsystem arm, ElevatorSubsystem elevator,
      XboxController controller) {
    m_arm = arm;
    m_elevator = elevator;
    m_controller = controller;

    addRequirements(arm, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("CLIMBING MANUAL ACTIVE", m_isUnlocked);
    m_controller.setRumble(RumbleType.kLeftRumble, 0.0);
    m_controller.setRumble(RumbleType.kRightRumble, 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_controller.getLeftBumperPressed()) {
      m_isUnlocked = !m_isUnlocked;
      //rumbleSequenceCommand().schedule();
      

      if (m_isUnlocked == false) {
        m_arm.setArmPercentOutput(0);
        m_elevator.stop();
      }
    }
    SmartDashboard.putBoolean("CLIMBING MANUAL ACTIVE", m_isUnlocked);

    if (m_isUnlocked) {
      // // TODO: check the arm multiplier for changes
      // double armJoyStickValue = m_controller.getRightY();
      // if (Math.abs(armJoyStickValue) > 0.1) {
      //   m_arm.setArmPercentOutput(armJoyStickValue * 0.3);
      // } else {
      //   m_arm.setArmPercentOutput(0);
      // }

      double elevatorJoyStickValue = m_controller.getLeftY();
      if (Math.abs(elevatorJoyStickValue) > 0.1) {
        m_elevator.brakeOff();
        m_elevator.setWinchPercentOutput(
            elevatorJoyStickValue * Constants.ElevatorConstants.elevatorSpeedMultiplier);
      } else {
        m_elevator.stop();
      }

      double pov = m_controller.getPOV();

      if(pov == 0){
        m_arm.setArmAngle(Constants.ArmConstants.CLIMBING_MIDDLE_ANGLE);
      } else if(pov == 90){
        m_arm.setArmAngle(Constants.ArmConstants.CLIMBING_FORWARD_ANGLE);
      } else if(pov == 180){
        m_arm.setArmAngle(Constants.ArmConstants.CLIMBING_NEXT_BAR_ANGLE);
      } else if(pov == 270){
        m_arm.setArmAngle(Constants.ArmConstants.CLIMBING_BACK_ANGLE);
      }
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("CLIMBING MANUAL ACTIVE", false);

    m_arm.setArmPercentOutput(0);
    m_elevator.setWinchPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void rumbleSequence(){
    m_controller.setRumble(RumbleType.kLeftRumble, 0.3);
    m_controller.setRumble(RumbleType.kRightRumble, 0.3);
  }
}
