// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.canId;

public class ArmSubsystem extends SubsystemBase {
  TalonFX m_armLeadMotor = new TalonFX(canId.CANID19_ARM_LEAD_MOTOR);
  TalonFX m_armFollowMotor = new TalonFX(canId.CANID20_ARM_FOLLOW_MOTOR);

  //check if this is canId
  CANSparkMax m_sparkMax = new CANSparkMax(1, MotorType.kBrushless);
  SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
  RelativeEncoder m_throughBoreEncoder;

  SparkMaxPIDController m_armPID;

  //double m_rotations = 20;
  public ArmSubsystem() {
    m_armLeadMotor.setNeutralMode(NeutralMode.Brake);
    m_armLeadMotor.setNeutralMode(NeutralMode.Brake);

    m_throughBoreEncoder = m_sparkMax.getAlternateEncoder(kAltEncType, 8192);
    
    // m_armPID = m_sparkMax.getPIDController();
    // m_armPID.setFeedbackDevice(m_throughBoreEncoder);
    // m_armPID.setOutputRange(-1, 1);

    // m_armPID.setReference(m_rotations, CANSparkMax.ControlType.kPosition);
  }

  public void setArmPercentOutput(double percentage){
    m_armLeadMotor.set(ControlMode.PercentOutput, percentage);
  }

  public void zeroEncoder(){
    m_throughBoreEncoder.setPosition(0);
  }

  public double getEncoderValue(){
    return m_throughBoreEncoder.getPosition();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
