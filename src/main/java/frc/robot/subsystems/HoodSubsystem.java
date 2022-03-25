// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.team2930.lib.util.linearInterpolator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase {
  /** Creates a new HoodSubsystem. */

  private WPI_TalonFX hoodMotor = new WPI_TalonFX(Constants.CANIVOR_canId.CANID7_HOOD, Constants.CANIVOR_canId.name);

  //TODO: CHECK these values
  private double gearRatio = 1.0 / 84.0;
  private double ticksPerDegree = (gearRatio / 4096.0) * 360.0;

  // min and max from Beau
  private double minHoodAngle = 15.0;
  private double maxHoodAngle = 33.5;

  private static final int kPIDLoopIdx = 0;
  private static final int kSlotIdx = 0;
  private static final int kTimeoutMs = 20;

  private double m_currentAngle;
  private double m_desiredAngle;
  private boolean m_atDesiredAngle;

  private static final double kP = 0.21;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kF = 0.23;
  private static final double kIzone = 0;

  // Smoothing factor for motion control. 0 = trapazoidal, 1-8 for greater smoothing
  private static final int kSmoothing = 0;

  private linearInterpolator hoodInterpolator;
  private double distancesInchesWithHoodAngleDegrees[][] = {
    {52, 15},
  };
  
  
  public HoodSubsystem() {

    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20Talon%20FX%20(Falcon%20500)/MotionMagic/src/main/java/frc/robot/Robot.java
    hoodMotor.configFactoryDefault();

		hoodMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx,
				kTimeoutMs);
    
    // set deadband to super small 0.005 (0.5 %).
	  // The default deadband is 0.04 (4 %)
		hoodMotor.configNeutralDeadband(0.005, kTimeoutMs);

    // TalonFX integrated sensor is always in phase with the motor.
    hoodMotor.setSensorPhase(false);
		hoodMotor.setInverted(false);

    hoodMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
		hoodMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

    // TalonFXConfiguration config = new TalonFXConfiguration();
    // config.slot0.kP = kP;
		// config.slot0.kI = kI;
		// config.slot0.kD = kD;
    // config.slot0.kF = kF; 
    // config.slot0.integralZone = kIzone;
    // hoodMotor.configAllSettings(config, kTimeoutMs);

    /* Set Motion Magic gains in slot0 - see documentation */
		hoodMotor.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
		hoodMotor.config_kF(kSlotIdx, kF, kTimeoutMs);
		hoodMotor.config_kP(kSlotIdx, kP, kTimeoutMs);
		hoodMotor.config_kI(kSlotIdx, kI, kTimeoutMs);
		hoodMotor.config_kD(kSlotIdx, kD, kTimeoutMs);

    zeroEncoder();

		// Set acceleration and cruise velocity 
    // TODO: get these values from Howdybots JVN
    // https://docs.google.com/spreadsheets/d/1sOS_vM87iaKPZUFSJTqKqaFTxIl3Jj5OEwBgRxc-QGM/edit#gid=852230499
		hoodMotor.configMotionCruiseVelocity(9600, kTimeoutMs);
		hoodMotor.configMotionAcceleration(10000, kTimeoutMs);

    hoodMotor.configMotionSCurveStrength(kSmoothing);

		/* Zero the sensor once on robot boot up */
		hoodMotor.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);

    hoodInterpolator = new linearInterpolator(distancesInchesWithHoodAngleDegrees);
  }


  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    m_currentAngle = ticksToDegrees(hoodMotor.getSelectedSensorPosition());

    if (Math.abs(m_currentAngle - m_desiredAngle) <= 0.25){
      m_atDesiredAngle = true;
    }
    else {
      m_atDesiredAngle = false;
    }
    
    hoodMotor.set(TalonFXControlMode.MotionMagic, angleToTicks(m_desiredAngle));

    SmartDashboard.putNumber("Hood ticks", hoodMotor.getSelectedSensorPosition(kPIDLoopIdx));
    SmartDashboard.putNumber("Hood ticks/s", hoodMotor.getSelectedSensorVelocity(kPIDLoopIdx) * 10);
    SmartDashboard.putNumber("Hood RPM", hoodMotor.getSelectedSensorVelocity(kPIDLoopIdx) * 10 / 4096 * 60);
    SmartDashboard.putNumber("Hood angle", m_currentAngle);
    SmartDashboard.putNumber("Hood desired angle", m_desiredAngle);
    SmartDashboard.putNumber("Hood motor output", hoodMotor.getMotorOutputPercent());
    SmartDashboard.putBoolean("Hood at desired angle", m_atDesiredAngle);

  }

  public double getCurrentAngle() {
    return m_currentAngle;
  }

  /**
   * zeroEncoder() - set the current position to be zero ticks
   */
  private void zeroEncoder(){
    /* Zero the sensor once on robot boot up */
		hoodMotor.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
  }

  public void setDesiredAngle(double angle) {
    if (angle < minHoodAngle) {
      angle = minHoodAngle;
    } else if (angle > maxHoodAngle) {
      angle = maxHoodAngle;
    }
    m_desiredAngle = angle;
  }

  public boolean isAtAngle(){
    return m_atDesiredAngle;
  }

  public double ticksToDegrees(double ticks) {

    return (ticks / ticksPerDegree) + minHoodAngle;
  }

  public double angleToTicks(double angle) {

    return (angle - minHoodAngle) * ticksPerDegree;
  }

  public double getAngleForDistance(double distanceFeet){
    return hoodInterpolator.getInterpolatedValue(distanceFeet * 12.0);
  }

}
