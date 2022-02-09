// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.BreakIterator;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

  private TalonFX winch_lead_talon = new TalonFX(Constants.canId.CANID9_ELEVATOR_LEAD_TALON);
  private TalonFX winch_follow_talon = new TalonFX(Constants.canId.CANID10_ELEVATOR_FOLLOW_TALON);
  private Solenoid frictionBrakeSolenoid =
      new Solenoid(PneumaticsModuleType.REVPH, Constants.canId.CANID8_FRICTION_BRAKE_SOLENOID);
  private final double gearRatio =  0.074;
  private final double winchDiameter_inches = 1.30;   // 1.25 diameter + string windings
  private final double winchCircumference = Math.PI * winchDiameter_inches;
  private double heightSetpointInches = 0.0;
  private double toleranceInches = 0.2;
  private double StartingTicks = 0;
  private final double ticks2distance = gearRatio * winchCircumference / 4096;
  
  public ElevatorSubsystem() {
    winch_lead_talon.configFactoryDefault();
    winch_follow_talon.configFactoryDefault();
    
    TalonFXConfiguration leadConfig = new TalonFXConfiguration();
    TalonFXConfiguration followConfig = new TalonFXConfiguration();

    leadConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

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
    winch_lead_talon.configForwardSoftLimitThreshold(18.0 / ticks2distance);
    winch_follow_talon.configForwardSoftLimitThreshold(18.0 / ticks2distance);
    winch_lead_talon.configForwardSoftLimitEnable(true);
    winch_follow_talon.configForwardSoftLimitEnable(true);

    winch_follow_talon.follow(winch_lead_talon);
    // follow motor turns in the same direction as the lead motor

    // TODO: add limit switches for full down
    // TODO: add soft limits for full up
    // see https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/limit-switch.html

    // NOTE: when we power up, we expect the elevator to be full down, triggering the lower limit switch.
    // if not, we need to move the elevator down to the lower limit switch (VERY SLOWLY).
    // hitting either limit switch must stop the elevator.
    // details on elevator motors and gearing can be found here:
    // https://docs.google.com/spreadsheets/d/1sOS_vM87iaKPZUFSJTqKqaFTxIl3Jj5OEwBgRxc-QGM/edit?usp=sharing
    // this also has suggested kP, and trapezoidal velocity profile constants.

    // See:
    // https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html
    // Example code:
    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20Talon%20FX%20(Falcon%20500)/PositionClosedLoop_AuxFeedForward/src/main/java/frc/robot/Robot.java


    // TODO: check if this is the right section to activate the default state of frictionBrakeSolenoid
    StartingTicks = winch_lead_talon.getSelectedSensorPosition();
    brakeOn();
  }

  public void setElevatorHeight(double heightInches) {
    heightSetpointInches = heightInches;
    // TODO: tell lead motor what the new setpoint. calculate encoder ticks from inches.
    // winch_lead_talon.set(TalonFXControlMode.Position, ticks);
  }

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
    return (winch_lead_talon.getSelectedSensorPosition()-StartingTicks)*ticks2distance;

  }

  public void zeroHeight() {
    // This is called when the elevator triggers the lower limit switch.
    // This needs to be done by a command, that runs the elevator to the lower limit switch. VERY
    // SLOWLY.
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

  public void stop() {
    brakeOn();
    setWinchPercentOutput(0.0);
  }

  public void brakeOff(){
    frictionBrakeSolenoid.set(false);
  }

  public void brakeOn(){
    frictionBrakeSolenoid.set(true);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Winch_RPM", elevatorEncoder.getVelocity());
    // TODO: put elevator distance traveled on SmartDashboard
    // TODO: put limit switch status on SmartDashboard
  }
}
