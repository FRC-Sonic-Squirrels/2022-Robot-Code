// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.team2930.lib.util.linearInterpolator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase {
  /** Creates a new HoodSubsystem. */

  private WPI_TalonFX hoodMotor = new WPI_TalonFX(Constants.CANIVOR_canId.CANID7_HOOD, Constants.CANIVOR_canId.name);

  private double gearRatio = 1.0 / 84.0;
  private double ticksToDegree = (gearRatio / 2048) * 360.0;



  // min and max from Beau
  private double minHoodAngleDeg = 15.0;
  private double maxHoodAngleDeg = 33.0; //actual is 33 

  private static final int kPIDLoopIdx = 0;
  private static final int kSlotIdx = 0;
  private static final int kTimeoutMs = 20;

  private double currentAngleDeg;
  private double desiredAngleDeg;
  private boolean atDesiredAngle;
  private double toleranceDegrees = 0.35;
  private boolean zeroed = false;
  private double percentOutput = 0.0;

  private static final double kP = 0.25;  // try 0.21 for faster
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kF = 0.0;
  private static final double kIzone = 100;

  // Smoothing factor for motion control. 0 = trapezoidal, 1-8 for greater smoothing
  private static final int kSmoothing = 4;

  private linearInterpolator hoodInterpolator;

  ArrayList<WPI_TalonFX> m_allMotors;

  
  public HoodSubsystem() {

    m_allMotors.add(hoodMotor);
    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20Talon%20FX%20(Falcon%20500)/MotionMagic/src/main/java/frc/robot/Robot.java
    hoodMotor.configFactoryDefault();

		hoodMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx,
				kTimeoutMs);
    
    // set deadband to 2%. The default deadband is 0.04 (4 %)
		// hoodMotor.configNeutralDeadband(0.02, kTimeoutMs);

    // TalonFX integrated sensor is always in phase with the motor.
    hoodMotor.setSensorPhase(false);

    // flip the motor direction
		hoodMotor.setInverted(true);

    hoodMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
    //hoodMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

    /* Set Motion Magic gains in slot0 - see documentation */
		hoodMotor.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
		hoodMotor.config_kF(kSlotIdx, kF, kTimeoutMs);
		hoodMotor.config_kP(kSlotIdx, kP, kTimeoutMs);
		hoodMotor.config_kI(kSlotIdx, kI, kTimeoutMs);
		hoodMotor.config_kD(kSlotIdx, kD, kTimeoutMs);
    hoodMotor.config_IntegralZone(kSlotIdx, kIzone, kTimeoutMs);

		// Set acceleration and cruise velocity 
    // TODO: get these values from Howdybots JVN
    // https://docs.google.com/spreadsheets/d/1sOS_vM87iaKPZUFSJTqKqaFTxIl3Jj5OEwBgRxc-QGM/edit#gid=852230499
		// hoodMotor.configMotionCruiseVelocity(9600, kTimeoutMs);
		// hoodMotor.configMotionAcceleration(10000, kTimeoutMs);
    // hoodMotor.configMotionSCurveStrength(kSmoothing);

    // JVN predicts a max of 1.6A needed
    // Motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 5.0, 10.0, 0.1));
    
    // reduce max output for testing
    hoodMotor.configPeakOutputForward(0.2, kTimeoutMs);
		hoodMotor.configPeakOutputReverse(-0.2, kTimeoutMs);

    //hoodMotor.configClosedloopRamp(0.5, kTimeoutMs);

    // set soft limit on reverse movement (Up)
    hoodMotor.configForwardSoftLimitThreshold(angleToTicks(maxHoodAngleDeg - 0.25));
    hoodMotor.configForwardSoftLimitEnable(true);

    // config hard limit switch for full down position
    hoodMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        LimitSwitchNormal.NormallyOpen, 0);

    hoodMotor.setNeutralMode(NeutralMode.Coast);
  
    hoodInterpolator = new linearInterpolator(Constants.hoodAngleDegreesTable);

    desiredAngleDeg = minHoodAngleDeg;
    zeroEncoder();
  }


  @Override
  public void periodic() {

    // check if we triggered lower limit switch, and reset elevator to zero
    if(atLowerLimit()) {
      if (! zeroed) {
        // only zero height once per time hitting limit switch
        zeroEncoder();
        zeroed = true;
      }
    }
    else {
      // not currently on limit switch, zero again next time we hit limit switch
      zeroed = false;
    }

    // percent output OR position control, not both
    if (Math.abs(percentOutput) > 0.05) {
      hoodMotor.set(ControlMode.PercentOutput, percentOutput);
    }
    else {
      currentAngleDeg = ticksToDegrees(hoodMotor.getSelectedSensorPosition());
      hoodMotor.set(TalonFXControlMode.Position, angleToTicks(desiredAngleDeg));
    }

    if (Math.abs(currentAngleDeg - desiredAngleDeg) <= toleranceDegrees){
      atDesiredAngle = true;
    }
    else {
      atDesiredAngle = false;
    } 

    // SmartDashboard.putNumber("Hood ticks", hoodMotor.getSelectedSensorPosition(kPIDLoopIdx));
    // SmartDashboard.putNumber("Hood ticks per s", hoodMotor.getSelectedSensorVelocity(kPIDLoopIdx) * 10);
    // SmartDashboard.putNumber("Hood RPM", hoodMotor.getSelectedSensorVelocity(kPIDLoopIdx) * 10 / 4096 * 60);
    SmartDashboard.putNumber("Hood angle", currentAngleDeg);
    SmartDashboard.putNumber("Hood desired angle", desiredAngleDeg);
    SmartDashboard.putBoolean("Hood at angle", atDesiredAngle);
    // SmartDashboard.putNumber("Hood motor output", hoodMotor.getMotorOutputPercent());
    // SmartDashboard.putNumber("Hood closed loop error", hoodMotor.getClosedLoopError(kPIDLoopIdx));
    SmartDashboard.putBoolean("Hood limit", atLowerLimit());

  }

  /**
   * getCurrentAngle() - return current hood angle in degrees
   */
  public double getCurrentAngle() {
    return currentAngleDeg;
  }

  /**
   * zeroEncoder() - set the current position to be zero ticks
   */
  private void zeroEncoder(){
    /* Zero the sensor once on robot boot up */
		hoodMotor.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
  }

  /**
   * atLowerLimit() returns true if the lower limit switch is triggered.
   */
  public boolean atLowerLimit() {
    return (1 == hoodMotor.isRevLimitSwitchClosed());
  }

  public void setMinAngle() {
    desiredAngleDeg = minHoodAngleDeg;
  }

  public void setAngleDegrees(double angle) {
    percentOutput = 0.0;
    if (angle < minHoodAngleDeg) {
      angle = minHoodAngleDeg;
    } else if (angle > maxHoodAngleDeg) {
      angle = maxHoodAngleDeg;
    }
    desiredAngleDeg = angle;
  }

  public boolean isAtAngle(){
    return atDesiredAngle;
  }

  public double ticksToDegrees(double ticks) {
    return (ticks * ticksToDegree) + minHoodAngleDeg;
  }

  public double angleToTicks(double angle) {
    return (angle - minHoodAngleDeg) / ticksToDegree;
  }

  /**
   * getAngleFromDistance() - return the hood angle in degrees for shooting a given distance
   * 
   * @param distanceFeet
   * @return
   */
  public double getAngleForDistanceFeet(double distanceFeet){
    // interpolator is in inches, so convert to inches
    return hoodInterpolator.getInterpolatedValue(distanceFeet * 12.0);
  }

  /**
   * 
   * @param output
   */
  public void setPercentOutput(double output) {
    percentOutput = output;
    hoodMotor.set(ControlMode.PercentOutput, output);
  }

  public ArrayList<WPI_TalonFX> getAllMotors(){
    return m_allMotors;
  }

}
