// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.elevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  //FIXME: Find actual module type 
  private DoubleSolenoid elevatorDeploySolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM ,elevatorConstants.deploySolenoid1, elevatorConstants.deploySolenoid2);
 
  //FIXME: find actual module type
  private Solenoid brakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, elevatorConstants.brakeSolenoid);
  //private CANSparkMax elevatorWinchP = new CANSparkMax(elevatorConstants.elevatorWinch, MotorType.kBrushless);
  private WPI_VictorSPX elevatorWinchC = new WPI_VictorSPX(elevatorConstants.elevatorWinch);
  //private final CANEncoder elevatorEncoder = elevatorWinchP.getEncoder(EncoderType.kHallSensor, 2048);
  private boolean elevatorDeployed = false;

  public ElevatorSubsystem() {
    elevatorDeploySolenoid.set(Value.kReverse);
    //elevatorWinchP.restoreFactoryDefaults();
    //elevatorWinchP.setIdleMode(CANSparkMax.IdleMode.kBrake);
    elevatorWinchC.configFactoryDefault();
    elevatorWinchC.setNeutralMode(NeutralMode.Brake);
  }

  public void deployElevator() {
    setElevatorDeployed(true);
    elevatorDeploySolenoid.set(Value.kForward);
    // RobotContainer.m_limelight.setCAMMode(1);
    // RobotContainer.m_limelight.setLEDMode(1);
  }

  public void retractElevator() {
    setElevatorDeployed(false);
    elevatorDeploySolenoid.set(Value.kReverse);
    // RobotContainer.m_limelight.setCAMMode(0);
  }

  public void brakeOff() {
    brakeSolenoid.set(true);
  }

  public void brakeOn() {
    brakeSolenoid.set(false);
  }

  public void setWinchPercentOutput(double Percent) {
    elevatorWinchC.set(ControlMode.PercentOutput, Percent);
  }

  public void stop() {
    brakeOn();
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
