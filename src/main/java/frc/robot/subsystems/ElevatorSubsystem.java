// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.team2930.lib.util.MotorUtils;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


// Details on the TalonFX motion profile control can be found here:
// https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html
// Example code:
// https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20Talon%20FX%20(Falcon%20500)/PositionClosedLoop_AuxFeedForward/src/main/java/frc/robot/Robot.java


public class ElevatorSubsystem extends SubsystemBase {

  private WPI_TalonFX winch_lead_talon = new WPI_TalonFX(Constants.canId.CANID9_ELEVATOR_LEAD_TALON);
  private WPI_TalonFX winch_follow_talon = new WPI_TalonFX(Constants.canId.CANID10_ELEVATOR_FOLLOW_TALON);
  private Solenoid frictionBrakeSolenoid =
      new Solenoid(PneumaticsModuleType.REVPH, Constants.pneumatics.channel_14_friction_break);
  private final double gearRatio =  0.074;
  private final double winchDiameter_inches = 1.30;   // 1.25 diameter + string windings
  private final double winchCircumference = Math.PI * winchDiameter_inches;
  private final double maxExtensionInches = 18.0;
  private double heightSetpointInches = 0.0;
  private double toleranceInches = 0.2;
  private double StartingTicks = 0;
  private double feedForwardClimbing = 0.025734; // from JVM calculator
  private double feedForwardDescending = 0.001;  // TODO: this is just a guess
  private final double ticks2distance = gearRatio * winchCircumference / 4096;
  
  public ElevatorSubsystem() {
    winch_lead_talon.configFactoryDefault();
    winch_follow_talon.configFactoryDefault();
    
    TalonFXConfiguration leadConfig = new TalonFXConfiguration();

    leadConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

    
    // Details on elevator motors, gearing and calculated kP and kFF are here
    // https://docs.google.com/spreadsheets/d/1sOS_vM87iaKPZUFSJTqKqaFTxIl3Jj5OEwBgRxc-QGM/edit?usp=sharing
    // this also has suggest trapezoidal velocity profile constants.
    leadConfig.slot0.kF = 0.025734; 
		leadConfig.slot0.kP = 0.054836;
		leadConfig.slot0.kI = 0.0;
		leadConfig.slot0.kD = 0.0;
		leadConfig.slot0.integralZone = 0.0;
		leadConfig.slot0.closedLoopPeakOutput = 1.0;

    leadConfig.motionAcceleration = 20521;    //  20521 ticks/100ms     = 11 in/s
		leadConfig.motionCruiseVelocity = 20521;  //  20521 ticks/100ms/sec = 11 in/s^2

    // set config
    winch_lead_talon.configAllSettings(leadConfig);

    winch_lead_talon.setNeutralMode(NeutralMode.Brake);
    winch_follow_talon.setNeutralMode(NeutralMode.Brake);

    // set soft limit on forward movement
    winch_lead_talon.configForwardSoftLimitThreshold(maxExtensionInches / ticks2distance);
    winch_follow_talon.configForwardSoftLimitThreshold(18.0 / ticks2distance);
    winch_lead_talon.configForwardSoftLimitEnable(true);
    winch_follow_talon.configForwardSoftLimitEnable(true);

    // config hard limit switch for full down position
    winch_lead_talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        LimitSwitchNormal.NormallyOpen, 0);

    winch_follow_talon.follow(winch_lead_talon);
    winch_follow_talon.setInverted(false);

    MotorUtils.setCtreStatusSlow(winch_follow_talon);

    // NOTE: when we power up, we expect the elevator to be full down, triggering the lower limit switch.
    // if not, we need to move the elevator down to the lower limit switch (VERY SLOWLY).
    // hitting either limit switch must stop the elevator.

    StartingTicks = winch_lead_talon.getSelectedSensorPosition();
    brakeOn();
  }

  public void setElevatorHeight(double heightInches) {
    if (heightInches < 0.0) {
      heightInches = 0.0;
    }

    if (heightInches <= heightSetpointInches) {
      // lifting up robot, use more feed forward
      winch_lead_talon.set(TalonFXControlMode.Position, heightInches / ticks2distance,
          DemandType.ArbitraryFeedForward, feedForwardClimbing);
    } else {
      // lowering robot, use less feed forward
      winch_lead_talon.set(TalonFXControlMode.Position, heightInches / ticks2distance,
          DemandType.ArbitraryFeedForward, feedForwardDescending);
    }
    // NOTE: this is how without arbitrary feed forward
    // winch_lead_talon.set(TalonFXControlMode.Position, heightInches / ticks2distance);

    heightSetpointInches = heightInches;
  }

  /**
   * @return true if withing tolerance of  target height
   */
  public boolean isAtHeight(double heightInches) {
    return (Math.abs(heightInches - getHeightInches()) < toleranceInches);
  }

  /**
   * isAtHeight() check if the elevator is at the target height.
   * 
   * @return true if the elevator is at the height setpoint
   */
  public boolean isAtHeight() {
    return (Math.abs(heightSetpointInches - getHeightInches()) < toleranceInches);
  }

  /**
   * getHeightInches() returns the current height of the elevator in inches.
   * 
   * @return height of the elevator in inches
   */
  public double getHeightInches() {
    return (winch_lead_talon.getSelectedSensorPosition() - StartingTicks) * ticks2distance;
  }

  /**
   * zeroHeight() resets the starting position of the elevator to the current height.
   *
   * This is called when the elevator triggers the lower limit switch. This needs to be done by a
   * command, that runs the elevator to the lower limit switch. VERY SLOWLY.
   */
  public void zeroHeight() {
    StartingTicks = winch_lead_talon.getSelectedSensorPosition();
  }

  /**
   * Manually run elevator motors. USE WITH CAUTION.
   */
  public void setWinchPercentOutput(double percent) {
    if (percent != 0.0) {
      brakeOff();
    }
    else {
      brakeOn();
    }
    winch_lead_talon.set(ControlMode.PercentOutput, percent);
  }

  /**
   * stop() stops the elevator motors and sets the brake.
   */
  public void stop() {
    brakeOn();
    setWinchPercentOutput(0.0);
  }

  /**
   * brakeOn() turns on the brake.
   */
  public void brakeOff(){
    frictionBrakeSolenoid.set(false);
  }

  /**
   * brakeOff() turns off the brake.
   */
  public void brakeOn(){
    frictionBrakeSolenoid.set(true);
  }

  /**
   * atLowerLimit() returns true if the lower limit switch is triggered.
   */
  public boolean atLowerLimit() {
    return (1 == winch_lead_talon.isRevLimitSwitchClosed());
  }

  @Override
  public void periodic() {
    // check if we triggered lower limit switch, and reset elevator to zero
    if(atLowerLimit()){
      zeroHeight();
    }
    SmartDashboard.putNumber("Elev_Subsystem Height (inches)", getHeightInches());
    SmartDashboard.putNumber("Elev_Subsystem Vel (inches/s)", ticks2distance * winch_lead_talon.getSelectedSensorVelocity() / 10.0);
    SmartDashboard.putNumber("Elev_Subsystem SetPoint", heightSetpointInches);
    SmartDashboard.putNumber("Elev_Subsystem Error", heightSetpointInches - getHeightInches());
    SmartDashboard.putBoolean("Elev_Subsystem limit", atLowerLimit());
    SmartDashboard.putNumber("Elev_Subsystem %output", winch_lead_talon.getMotorOutputPercent());
    SmartDashboard.putNumber("Elev_Subsystem Current", winch_lead_talon.getSupplyCurrent());
  }
}
