// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.BreakIterator;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

  private TalonFX winch_lead_talon = new TalonFX(Constants.canId.CANID9_ELEVATOR_LEAD_TALON);
  private TalonFX winch_follow_talon = new TalonFX(Constants.canId.CANID10_ELEVATOR_FOLLOW_TALON);
  private Solenoid frictionBrakeSolenoid =
      new Solenoid(PneumaticsModuleType.REVPH, Constants.canId.CANID8_FRICTION_BRAKE_SOLENOID);
  private final double gearRatio =  0.074;
  private final double winchDiameter_inches = 1.25;
  private double heightSetpointInches = 0.0;
  private double toleranceInches = 0.2;;

  public ElevatorSubsystem() {
    winch_lead_talon.configFactoryDefault();
    winch_follow_talon.configFactoryDefault();

    winch_lead_talon.setNeutralMode(NeutralMode.Brake);
    winch_follow_talon.setNeutralMode(NeutralMode.Brake);

    winch_follow_talon.follow(winch_lead_talon);
    // TODO: check to see if follow motor is reversed from lead motor

    // TODO: add 2 limit switches for full down and full up
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
    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20Talon%20FX%20(Falcon%20500)/MotionMagic_ArbFeedForward/src/main/java/frc/robot/Robot.java


    // TODO: check if this is the right section to activate the default state of frictionBrakeSolenoid
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
    // TODO: calculate height of elevator from encoder
    return 0.0;
  }

  public void zeroHeight() {
    // TODO: zero the height of the elevator. Call this when resetting encoder.
    // This is called when the elevator triggers the lower limit switch.
    // This needs to be done by a command, that runs the elevator to the lower limit switch. VERY
    // SLOWLY.
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
