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

  private double m_configP = 0;
  private double m_configI = 0;
  private double m_configD = 0;
  private double m_configF = 0;
  private double m_configIZ = 0;

  // lower number here, slows the rate of change and decreases the power spike 
  private double m_rate_RPMpersecond = 2500;
  private SlewRateLimiter m_rateLimiter;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    adjustPID();
  }

  @Override
  public void periodic() {
    //this is for testing and tuning the pid
    setPIDFromSmartDashboard();

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

  private void setPIDFromSmartDashboard(){
    double P = SmartDashboard.getNumber("Shooter_K", 0);
    double I = SmartDashboard.getNumber("Shooter_I", 0);
    double D = SmartDashboard.getNumber("Shooter_D", 0);
    double F = SmartDashboard.getNumber("Shooter_F", 0);
    double IZ = SmartDashboard.getNumber("Shooter_IZ", 0);

    boolean hasChanged = false;
    if(m_configP != P) {m_configP = P; hasChanged = true;}
    if(m_configI != I) {m_configI = I; hasChanged = true;}
    if(m_configD != D) {m_configD = D; hasChanged = true;}
    if(m_configF != F) {m_configF = F; hasChanged = true;}
    if(m_configIZ != IZ) {m_configIZ = IZ; hasChanged = true;}

    if(hasChanged){
      adjustPID();
    }
  }

  private void adjustPID(){
    flywheel.config_kP(0, m_configP);
    flywheel.config_kI(0, m_configI);
    flywheel.config_kD(0, m_configD);
    flywheel.config_kF(0, m_configF);
    flywheel.config_IntegralZone(0, m_configIZ);
  }
}
