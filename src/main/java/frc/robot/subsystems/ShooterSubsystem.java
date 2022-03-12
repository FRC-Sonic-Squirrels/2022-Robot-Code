// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.team2930.lib.util.MotorUtils;
import com.team2930.lib.util.linearInterpolator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  enum ShooterMode {
    STATIC, DYNAMIC
  };

  private WPI_TalonFX flywheel_lead= new WPI_TalonFX(Constants.canId.CANID16_flywheel_lead);
  private WPI_TalonFX flywheel_follow = new WPI_TalonFX(Constants.canId.CANID17_flywheel_follow);
  private TalonFXSensorCollection m_encoder;
  private double kMaxOutput, kMinOutput;
  private double m_desiredRPM = 0;
  private boolean m_atSpeed = false;
  private linearInterpolator m_lt_feet;
  private int m_idleRPM = 2000;
  private double m_currentRPM = 0;
  private double m_error = 0;
  private double m_max_RPM_error = 15;
  private final double RPMtoTicks = 2048 / 600;
  private double m_testingStaticRPM = 0;

  private double m_configP = 0.2;
  private double m_configI = 0.001;
  private double m_configD = 0.0;
  private double m_configF = 0.052;
  private double m_configIZ = 100;

  // lower number here, slows the rate of change and decreases the power spike
  private double m_rate_RPMperSecond = 6000;
  private SlewRateLimiter m_rateLimiter = new SlewRateLimiter(m_rate_RPMperSecond);

  // TODO: find actual values
  private double shooterDistances[][] = {
    {4.0, 3700},  // 4 feet 
    {4.7, 3800},
    {5.0, 3850},  // 5 feet
    {5.8, 3850},
    {6.6, 3900},
    {9.7, 4675},
    {11.0, 5000}, 
    {15.0, 5400}, // 15 feet
    {16.2, 5500},
    {20.0, 6000},  // 20 feet
    {23.3, 6000},
    {25.0, 6200}
  };

  //TODO: add shooting, idle and stop enums use them for logic for setting rpm when shooting, setting motor voltage to 0 when idle and slowing/setting rpm for when stopping

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    // TODO: go over this and confirm motors are set up correctly. check against 2021 code
    flywheel_lead.configFactoryDefault();
    flywheel_follow.configFactoryDefault();

    flywheel_lead.setNeutralMode(NeutralMode.Coast);
    flywheel_follow.setNeutralMode(NeutralMode.Coast);

    flywheel_lead.configVoltageCompSaturation(11.0);
    flywheel_lead.enableVoltageCompensation(true);
    flywheel_follow.configVoltageCompSaturation(11.0);
    flywheel_follow.enableVoltageCompensation(true);

    flywheel_lead.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    m_encoder = flywheel_lead.getSensorCollection();

    flywheel_lead.setInverted(false);

    flywheel_follow.follow(flywheel_lead);
    flywheel_follow.setInverted(true);

    // Build the linear Interpolator
    m_lt_feet = new linearInterpolator(shooterDistances);

    MotorUtils.setCtreStatusSlow(flywheel_follow);

    setPID();
  }

  public void updateTestingRPM() {
    m_testingStaticRPM = SmartDashboard.getNumber("static flywheel speed", m_testingStaticRPM);
  }

  public void stop() {
    m_desiredRPM = 0;
    flywheel_lead.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // this is for testing and tuning the pid
    // setPIDFromSmartDashboard();
    updateTestingRPM();

    double setPoint = 0;
    m_currentRPM = m_encoder.getIntegratedSensorVelocity() / RPMtoTicks;
    m_error = m_currentRPM - m_desiredRPM;

    if (Math.abs(m_error) <= m_max_RPM_error) {
      m_atSpeed = true;
    } else {
      m_atSpeed = false;
    }

    setPoint = m_rateLimiter.calculate(m_desiredRPM);
    if (m_desiredRPM < setPoint) {
      // we don't rate reduce slowing the robot
      setPoint = m_desiredRPM;
    }

    

    if (m_desiredRPM == 0) {
      // special case, turn off power to flywheel
      flywheel_lead.set(ControlMode.PercentOutput, 0);
    }
    else {
      flywheel_lead.set(ControlMode.Velocity, setPoint * RPMtoTicks);
    }

    SmartDashboard.putNumber("Shooter RPM set point", setPoint);
    SmartDashboard.putNumber("Shooter RPM error", m_error);
    SmartDashboard.putBoolean("Shooter isAtSpeed", m_atSpeed);
    SmartDashboard.putNumber("Shooter m_CurrentRPM", m_currentRPM);
    SmartDashboard.putNumber("Shooter m_desiredRPM", m_desiredRPM);
    SmartDashboard.putBoolean("Shooter m_atSpeed", m_atSpeed);
    SmartDashboard.putNumber("Shooter m_idleRPM", m_idleRPM);
    SmartDashboard.putNumber("Shooter m_error", m_error);
    SmartDashboard.putNumber("Shooter m_max_RPM_error", m_max_RPM_error);
    SmartDashboard.putNumber("Shooter RPMtoTicks", RPMtoTicks);
    SmartDashboard.putNumber("Shooter m_testingStaticRPM", m_testingStaticRPM);
    SmartDashboard.putNumber("Shooter  m_configI", m_configI);
    SmartDashboard.putNumber("Shooter  m_configD", m_configD);
    SmartDashboard.putNumber("Shooter  m_configF", m_configF);
    SmartDashboard.putNumber("Shooter  m_configIZ", m_configIZ);
    SmartDashboard.putNumber("Shooter m_rate_RPMperSecond", m_rate_RPMperSecond);

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

  public double getRPMforDistanceFeet(double distanceFeet) {
    return m_lt_feet.getInterpolatedValue(distanceFeet);
  }

  private void setPIDFromSmartDashboard() {
    double P = SmartDashboard.getNumber("Shooter_K", 0);
    double I = SmartDashboard.getNumber("Shooter_I", 0);
    double D = SmartDashboard.getNumber("Shooter_D", 0);
    double F = SmartDashboard.getNumber("Shooter_F", 0);
    double IZ = SmartDashboard.getNumber("Shooter_IZ", 0);

    boolean hasChanged = false;
    if (m_configP != P) {
      m_configP = P;
      hasChanged = true;
    }
    if (m_configI != I) {
      m_configI = I;
      hasChanged = true;
    }
    if (m_configD != D) {
      m_configD = D;
      hasChanged = true;
    }
    if (m_configF != F) {
      m_configF = F;
      hasChanged = true;
    }
    if (m_configIZ != IZ) {
      m_configIZ = IZ;
      hasChanged = true;
    }

    if (hasChanged) {
      setPID();
    }
  }

  private void setPID() {
    flywheel_lead.config_kP(0, m_configP);
    flywheel_lead.config_kI(0, m_configI);
    flywheel_lead.config_kD(0, m_configD);
    flywheel_lead.config_kF(0, m_configF);
    flywheel_lead.config_IntegralZone(0, m_configIZ);
  }
}
