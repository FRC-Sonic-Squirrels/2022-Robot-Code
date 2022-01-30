// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.canId;

public class ArmSubsystem extends SubsystemBase {
  TalonFX m_armLeadMotor = new TalonFX(canId.CANID19_ARM_LEAD_MOTOR);
  TalonFX m_armFallowMotor = new TalonFX(canId.CANID20_ARM_FALLOW_MOTOR);

  //check if this is canId
  CANSparkMax m_sparkMax = new CANSparkMax(1, MotorType.kBrushless);
  RelativeEncoder m_throughBoreEncoder;
  
  //TODO: Find actual values these are made up
  PIDController armPID = new PIDController(3, 0.2, 0.03);

  public ArmSubsystem() {
    m_throughBoreEncoder = m_sparkMax.getEncoder();
    
    //TODO: do we need this, if so what value do we use?
    // 1 degree of wiggle room
    armPID.setTolerance(Math.PI/180); 
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
