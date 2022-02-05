// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private double targetRPM;
  private WPI_TalonFX flywheel = new WPI_TalonFX(Constants.canId.CANID7_FLYWHEEL);
  private TalonFXSensorCollection m_encoder;
  private double kMaxOutput, kMinOutput;
  private double m_desiredRPM = 0;
  private boolean m_atSpeed = false;
  //TODO: import linearInterpolator
  //private linearInterpolator m_lt_feet;
  private int m_idleRPM = 2000;
  private double m_currentRPM = 0;
  private double m_error = 0;
  private double m_max_RPM_error = 15;
  private final double RPMtoTicks = 2048 / 600;

  // lower number here, slows the rate of change and decreases the power spike 
  private double m_rate_RPMpersecond = 2500;
  private SlewRateLimiter m_rateLimiter;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_currentRPM = m_encoder.getIntegratedSensorVelocity() / RPMtoTicks;
    m_error = m_currentRPM - m_desiredRPM;

    //if (Math.abs(m_error) < m_max_RPM_error) {
    if (((m_error >= 0) && (m_error < m_max_RPM_error)) ||
        ((m_error < 0) && (m_error > -m_max_RPM_error))) {
      m_atSpeed = true;
    }
    else {
      m_atSpeed = false;  
    }

    double setPoint = m_rateLimiter.calculate(m_desiredRPM);
    if (m_desiredRPM < setPoint) {
      // we don't rate reduce slowing the robot
      setPoint = m_desiredRPM;
    }


    flywheel.set(ControlMode.Velocity, setPoint * RPMtoTicks);

    SmartDashboard.putNumber("RPM", m_currentRPM);
    SmartDashboard.putNumber("RPM set point", setPoint);
    SmartDashboard.putNumber("RPM error", m_error);
    SmartDashboard.putBoolean("isAtSpeed", m_atSpeed);
  }

  public void setFlywheelRPM(double rpm) {
    m_desiredRPM = rpm;
  }

  public double getCurrentRPM() {
    return m_currentRPM;
  }

  public boolean isAtDesiredRPM() {
    return m_atSpeed;
  }

  public double getDesiredRPM() {
    return m_desiredRPM;
  }

  public double getIdleRPM() {
    return m_idleRPM;
  }
}
