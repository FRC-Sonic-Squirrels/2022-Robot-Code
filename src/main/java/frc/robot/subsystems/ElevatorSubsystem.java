// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper.GearRatio;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

  private TalonFX winch_lead_talon = new TalonFX(Constants.canId.CANID9_ELEVATOR_LEAD_TALON);
  private TalonFX winch_follow_talon = new TalonFX(Constants.canId.CANID10_ELEVATOR_FOLLOW_TALON);
  private Solenoid frictionBrakeSolenoid =
      new Solenoid(PneumaticsModuleType.REVPH, Constants.canId.CANID8_FRICTION_BRAKE_SOLENOID);
  private final double gearRatio =  0.074;
  private final double winchDiameter_inches = 1.25;
  private boolean elevatorDeployed = false;
  


  public ElevatorSubsystem() {
    //elevatorWinchP.restoreFactoryDefaults();
    //elevatorWinchP.setIdleMode(CANSparkMax.IdleMode.kBrake);
    //elevatorWinchC.configFactoryDefault();
    //elevatorWinchC.setNeutralMode(NeutralMode.Brake);
    winch_lead_talon.setNeutralMode(NeutralMode.Brake);
    winch_follow_talon.follow(winch_lead_talon);

    // TODO: add 2 limit switches for full down and full up
    // see https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/limit-switch.html

    // NOTE: when we power up, we expect the elevator to be full down, triggering the lower limit switch.
    // if not, we need to move the elevator down to the lower limit switch (VERY SLOWLY).
    // hitting either limit switch must stop the elevator.
    // details on elevator motors and gearing can be found here:
    // https://docs.google.com/spreadsheets/d/1sOS_vM87iaKPZUFSJTqKqaFTxIl3Jj5OEwBgRxc-QGM/edit?usp=sharing
    // this also has suggested kP, and trapezoidal velocity profile constants.

    // See:
    // https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html
    // Example code:
    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20Talon%20FX%20(Falcon%20500)/MotionMagic_ArbFeedForward/src/main/java/frc/robot/Robot.java


    // TODO: check if this is the right section to activate the default state of frictionBrakeSolenoid
    frictionBrakeSolenoid.set(true);

  }

  public void deployElevator() {
    setElevatorDeployed(true);
    frictionBrakeSolenoid.set(false);
    // RobotContainer.m_limelight.setCAMMode(1);
    // RobotContainer.m_limelight.setLEDMode(1);
  }

  public void retractElevator() {
    setElevatorDeployed(false);
    frictionBrakeSolenoid.set(true);
    // RobotContainer.m_limelight.setCAMMode(0);
  }

  public void setWinchPercentOutput(double percent) {
    winch_lead_talon.set(ControlMode.PercentOutput, percent);
    winch_follow_talon.set(ControlMode.PercentOutput, percent);
  }

  public void stop() {
    frictionBrakeSolenoid.set(true);
    setWinchPercentOutput(0.0);
  }

  public void setElevatorDeployed(boolean state) {
    elevatorDeployed = state;
  }

  public boolean getElevatorDeployed() {
    return elevatorDeployed;
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Winch_RPM", elevatorEncoder.getVelocity());
  }
}
