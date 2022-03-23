// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.team2930.lib.util.linearInterpolator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase {
  /** Creates a new HoodSubsystem. */

  private WPI_TalonFX m_hood = new WPI_TalonFX(Constants.CANIVOR_canId.CANID7_HOOD);
  private TalonFXSensorCollection m_encoder;
  private double m_gearRatio;
  private double m_ticksPerDegree = 5.689;
  //TODO: fix the parameter values
  private PIDController m_pidController = new PIDController(0, 0, 0); 

  private double m_currentAngle;
  private double m_desiredAngle;
  private boolean m_atDesiredAngle;

  private linearInterpolator hoodInterpolator;
  private double distancesInchesWithHoodAngleDegrees[][] = {
    {52, 15},
  };
  
  
  public HoodSubsystem() {
    // Build the linear Interpolator
    hoodInterpolator = new linearInterpolator(distancesInchesWithHoodAngleDegrees);
  }

//set point

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_currentAngle = ticksToDegrees(m_encoder.getIntegratedSensorAbsolutePosition());

    if(Math.abs(m_currentAngle - m_desiredAngle)<=0.25){
      m_atDesiredAngle = true;
    }
    else{
      m_atDesiredAngle = false;
      double controllerOutput = m_pidController.calculate(m_currentAngle, m_desiredAngle);
      m_hood.set(ControlMode.PercentOutput, controllerOutput);
    }

    

  }

  public double getCurrentAngle(){
    return m_currentAngle;
  }

  public void setDesiredAngle(double angle){
    m_desiredAngle = angle;
  }

  public boolean isAtAngle(){
    return m_atDesiredAngle;
  }

  public double ticksToDegrees(double ticks){
    return ticks/m_ticksPerDegree;
  }

  public double getAngleForDistance(double distanceFeet){
    return hoodInterpolator.getInterpolatedValue(distanceFeet * 12.0);
  }

  public double angleToEncoder(double angle){
    return angle*m_ticksPerDegree;
  }

}
