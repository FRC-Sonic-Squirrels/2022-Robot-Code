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

  private TalonFX winch_main_talon = new TalonFX(Constants.canId.canId9_main_talon);
  private TalonFX winch_sub_talon = new TalonFX(Constants.canId.canId10_sub_talon);
  private Solenoid frictionBrakeSolenoid =
      new Solenoid(PneumaticsModuleType.REVPH, Constants.canId.canId11_friction_brake_solenoid);
  private final double gearRatio =  0.074;
  private final double winchDiameter_inches = 1.25;
  private boolean elevatorDeployed = false;
  


  public ElevatorSubsystem() {
    //elevatorWinchP.restoreFactoryDefaults();
    //elevatorWinchP.setIdleMode(CANSparkMax.IdleMode.kBrake);
    //elevatorWinchC.configFactoryDefault();
    //elevatorWinchC.setNeutralMode(NeutralMode.Brake);

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
    winch_main_talon.set(ControlMode.PercentOutput, percent);
    winch_sub_talon.set(ControlMode.PercentOutput, percent);
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
