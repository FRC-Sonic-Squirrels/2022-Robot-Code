// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.team2930.lib.util.linearInterpolator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIVOR_canId;
import frc.robot.Robot;

public class ShooterSubsystem extends SubsystemBase {

  public enum ShooterMode {
    STOP, IDLE, SHOOTING 
  };

  private ShooterMode m_mode = ShooterMode.STOP;

  private double m_idleRpm = Constants.ShooterConstants.IDLE;

  private Robot m_robot;

  private WPI_TalonFX flywheel_lead= new WPI_TalonFX(CANIVOR_canId.CANID16_flywheel_lead, CANIVOR_canId.name);
  private WPI_TalonFX flywheel_follow = new WPI_TalonFX(CANIVOR_canId.CANID17_flywheel_follow, CANIVOR_canId.name);
  private TalonFXSensorCollection m_encoder;
  private double m_desiredRPM = 0;
  private boolean m_atSpeed = false;
  private linearInterpolator RPMinterpolator;
  private double m_currentRPM = 0;
  private double m_error = 0;
  private double m_max_RPM_error = 30;
  private final double RPMtoTicks = 2048 / 600;

  private boolean autonPIDset = false;

  private double teleop_configP = 0.2;
  private double teleop_configI = 0.0003;
  private double teleop_configD = 0.3;
  private double teleop_configF = 0.051;
  private double teleop_configIZ = 3000;   // 12288 = 100 RPM Izone
  private double maxForwardOutput = 1.0;
  private double maxReverseOutput = -0.05;

  // NOTE: this was a hack to fix weirdness during autonomous
  // private double auton_configP = 0.25;
  // private double auton_configI = 0.002;
  // private double auton_configD = 0.0;
  // private double auton_configF = 0.06;
  // private double auton_configIZ = 100;

  // lower number here, slows the rate of change and decreases the power spike
  private double m_rate_RPMperSecond = 10000000;
  private SlewRateLimiter m_rateLimiter = new SlewRateLimiter(m_rate_RPMperSecond);

  // private PowerDistribution m_revPDH; 

  // private double tpowerPort5 = 0;
  // private double tpowerPort6 = 0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(Robot robot, PowerDistribution revPhd) {
    m_robot = robot;

    flywheel_lead.configFactoryDefault();
    flywheel_follow.configFactoryDefault();

    flywheel_lead.setNeutralMode(NeutralMode.Coast);
    flywheel_follow.setNeutralMode(NeutralMode.Coast);

    flywheel_lead.configVoltageCompSaturation(11.0);
    flywheel_lead.enableVoltageCompensation(true);
    flywheel_follow.configVoltageCompSaturation(11.0);
    flywheel_follow.enableVoltageCompensation(true);

    flywheel_lead.configPeakOutputForward(maxForwardOutput);
    flywheel_lead.configPeakOutputReverse(maxReverseOutput);

    flywheel_lead.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    m_encoder = flywheel_lead.getSensorCollection();

    flywheel_lead.setInverted(false);

    flywheel_follow.follow(flywheel_lead);
    flywheel_follow.setInverted(true);

    // Build the linear Interpolator
    RPMinterpolator = new linearInterpolator(Constants.flywheelRpmTable);
    
    // Be more responsive to changes in the RPM
    // https://docs.ctre-phoenix.com/en/stable/ch14_MCSensor.html#velocity-measurement-filter
    // defaults are 100ms, 64 samples
    flywheel_lead.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
    flywheel_lead.configVelocityMeasurementWindow(32);

    // MotorUtils.setCtreStatusSlow(flywheel_follow);

    SmartDashboard.putBoolean("SHOOTER USE IDLE RPM", true);

    setPIDteleop();

    //m_revPDH = revPhd;
  }

  @Override
  public void periodic() {
    // this was debug for testing how much power the flywheel draws at idle
    // double power5 = m_revPDH.getCurrent(5) * flywheel_lead.getMotorOutputVoltage();
    // double power6 = m_revPDH.getCurrent(6) * flywheel_follow.getMotorOutputVoltage();

    // SmartDashboard.putNumber("flywheel power 5", power5);
    // SmartDashboard.putNumber("flywheel power 6", power6);

    // tpowerPort5 = tpowerPort5 + power5;
    // tpowerPort6 = tpowerPort6 + power6;

    // SmartDashboard.putNumber("total flywheel power 5", tpowerPort5);
    // SmartDashboard.putNumber("total flywheel power 6",  tpowerPort6);

    // SmartDashboard.putNumber("total flywheel both", tpowerPort5 + tpowerPort6);

    double setPoint = 0;
    switch (m_mode) {
      case STOP:
        flywheel_lead.set(ControlMode.PercentOutput, 0);
        m_desiredRPM = 0; //need to update this because its used for determining isAtSpeed
        break;

      case IDLE: 
        //can turn off idle rpm from dashboard
        if( !SmartDashboard.getBoolean("SHOOTER USE IDLE RPM", false) ){
          m_mode = ShooterMode.STOP;
          break;
        }
        //our max motor output slowing down is 5% power this means slowing our flywheel is super slow 
        //instead if the flywheel is spinning super fast, cut power to lower its velocity faster 
        //this should mean we avoid the case where we wait for the shooter to slow down  

        //no abs bc value if current rpm is within 50 but less than idle rpm get it up to idle rpm 
        if( (m_currentRPM - m_idleRpm > 50) && (m_currentRPM > m_idleRpm) ) {
          flywheel_lead.set(ControlMode.PercentOutput, 0);
        } else {
          flywheel_lead.set(ControlMode.Velocity, m_idleRpm * RPMtoTicks);
        }

        m_desiredRPM = m_idleRpm; //need to update this because its used for determining isAtSpeed
        break;

      case SHOOTING: 
        setPoint = m_rateLimiter.calculate(m_desiredRPM);
        if (m_desiredRPM < setPoint) {
          // we don't rate reduce slowing the robot
          setPoint = m_desiredRPM;
        }

        flywheel_lead.set(ControlMode.Velocity, setPoint * RPMtoTicks);
        break;

      default:
        flywheel_lead.set(ControlMode.PercentOutput, 0);
        break;
    }

    // double setPoint = 0;
    m_currentRPM = m_encoder.getIntegratedSensorVelocity() / RPMtoTicks;
    m_error = m_currentRPM - m_desiredRPM;

    if (Math.abs(m_error) <= m_max_RPM_error && m_currentRPM > 100) {
      m_atSpeed = true;
    } else {
      m_atSpeed = false;
    }

    // setPoint = m_rateLimiter.calculate(m_desiredRPM);
    // if (m_desiredRPM < setPoint) {
    //   // we don't rate reduce slowing the robot
    //   setPoint = m_desiredRPM;
    // }

    // if (m_desiredRPM == 0) {
    //   // special case, turn off power to flywheel
    //   flywheel_lead.set(ControlMode.PercentOutput, 0);
    // }
    // else {
    //   flywheel_lead.set(ControlMode.Velocity, setPoint * RPMtoTicks);
    // }

    SmartDashboard.putNumber("Shooter RPM set point", setPoint);
    SmartDashboard.putNumber("Shooter RPM error", m_error);
    SmartDashboard.putBoolean("Shooter isAtSpeed", m_atSpeed);
    SmartDashboard.putNumber("Shooter m_CurrentRPM", m_currentRPM);
    SmartDashboard.putNumber("Shooter m_desiredRPM", m_desiredRPM);
    SmartDashboard.putNumber("Shooter m_error", m_error);

    SmartDashboard.putNumber("flywheel output percent", flywheel_lead.getMotorOutputPercent());
    SmartDashboard.putNumber("flywheel output voltage", flywheel_lead.getMotorOutputVoltage());
    SmartDashboard.putString("flywheel MODE", m_mode.name());
    // SmartDashboard.putNumber("Shooter m_max_RPM_error", m_max_RPM_error);
    // SmartDashboard.putBoolean("Shooter auton PID", autonPIDset);
    // SmartDashboard.putNumber("Shooter m_rate_RPMperSecond", m_rate_RPMperSecond);

  }

  public ShooterMode getMode(){
    return m_mode;
  }

  public void setFlywheelRPM(double rpm) {
    m_desiredRPM = rpm;
    m_mode = ShooterMode.SHOOTING;
    flywheel_lead.setIntegralAccumulator(0.0);
  }

  public void stop() {
    m_mode = ShooterMode.STOP;
  }

  public void idle(){
    m_mode = ShooterMode.IDLE;
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

  public double getRPMforDistanceFeet(double distanceFeet) {
    return RPMinterpolator.getInterpolatedValue(distanceFeet * 12.0);
  }

  private void setPIDteleop() {
    autonPIDset = false;
    flywheel_lead.config_kP(0, teleop_configP);
    flywheel_lead.config_kI(0, teleop_configI);
    flywheel_lead.config_kD(0, teleop_configD);
    flywheel_lead.config_kF(0, teleop_configF);
    flywheel_lead.config_IntegralZone(0, teleop_configIZ);
  }

}
