/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.currentLimits;
import static frc.robot.Constants.digitalIOConstants;
import static frc.robot.Constants.canId;

public class CargoSubsystem extends SubsystemBase {

  enum Mode {
    STOP,
    INTAKE,
    LOWERONLY,
    UPPERONLY,
    BOTH,
    REVERSE
  };

  private WPI_TalonFX UpperBelts;
  private WPI_TalonFX LowerBelts;
  // TODO: add color sensor for cargo in upper belts
  private DigitalInput lowerSensor = new DigitalInput(digitalIOConstants.dio0_indexerSensor1);
  private DigitalInput upperSensor = new DigitalInput(digitalIOConstants.dio1_indexerSensor2);
  private Mode mode = Mode.STOP;
  private int ballCount = 0;
  private double percentOutput = 0.7;

  public CargoSubsystem() {

    LowerBelts = new WPI_TalonFX(canId.CANID5_LOWER_BELTS);
    UpperBelts = new WPI_TalonFX(canId.CANID6_UPPER_BELTS);

    LowerBelts.configFactoryDefault();
    UpperBelts.configFactoryDefault();

    // reduce CAN traffic for motors (not using speed control)
    LowerBelts.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
    LowerBelts.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);
    UpperBelts.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
    UpperBelts.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);

    // Voltage limits, percent output is scaled to this new max
    LowerBelts.configVoltageCompSaturation(11);
    LowerBelts.enableVoltageCompensation(true);
    UpperBelts.configVoltageCompSaturation(11);
    UpperBelts.enableVoltageCompensation(true);

    // current limits
    LowerBelts.configSupplyCurrentLimit(currentLimits.m_currentlimitSecondary);
    UpperBelts.configSupplyCurrentLimit(currentLimits.m_currentlimitSecondary);

    // Brake mode
    LowerBelts.setNeutralMode(NeutralMode.Brake);
    UpperBelts.setNeutralMode(NeutralMode.Brake);

    // Invert
    LowerBelts.setInverted(false);
    UpperBelts.setInverted(false);

    LowerBelts.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    UpperBelts.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    
    //Set Ramp-Up
    //UpperBelts.configClosedloopRamp(0.1);
    //LowerBelts.configClosedloopRamp(0.1);

    // TODO: configure PID for lower and upper belts
    // Config PID values to control RPM
    LowerBelts.config_kP(0, 0.15, 10);
    LowerBelts.config_kI(0, 0.0, 10);
    LowerBelts.config_kD(0, 1.5, 10);
    LowerBelts.config_kF(0, 0.048, 10);

    UpperBelts.config_kP(0, 0.15, 10);
    UpperBelts.config_kI(0, 0.0, 10);
    UpperBelts.config_kD(0, 1.5, 10);
    UpperBelts.config_kF(0, 0.053, 10);

  }

  @Override
  public void periodic() {

    // TODO: logic on running and stopping belts must completely change for 2022
    // TODO: check the real percent outputs of the conveyor belts
    if (mode == Mode.STOP) {
      stopIndexer();
    } 
    else if (mode == Mode.INTAKE) {
      if (cargoInUpperBelts()) {
        stopUpperBelts();
        if (cargoInLowerBelts()) {
          stopLowerBelts();
        } else {
          setLowerBeltPercentOutput(percentOutput);
        }
      } else {
        setLowerBeltPercentOutput(percentOutput);
        setUpperBeltPercentOutput(percentOutput);
      }
      
    } 
    else if (mode == Mode.LOWERONLY) {
      stopUpperBelts();
      setLowerBeltPercentOutput(percentOutput);
    } 
    else if (mode == Mode.UPPERONLY) {
      stopLowerBelts();
      setUpperBeltPercentOutput(percentOutput);
    } 
    else if (mode == Mode.BOTH) {
      // Normal, non-eject mode
      SmartDashboard.putNumber("Eject State", 0);

      // shoot mode releases the upper cargo, then moves the lower cargo to the top
      setUpperBeltPercentOutput(percentOutput);
      setLowerBeltPercentOutput(percentOutput);
    } 
    else if (mode == Mode.REVERSE) {
      setUpperBeltPercentOutput(-percentOutput); //negate percent output to make belts go in reverse
      setLowerBeltPercentOutput(-percentOutput);
    }
    else {
      stopIndexer();
    }

    // if (cargoInLowerBelts() && cargoInUpperBelts()) {
    //   setUpperOnlyMode();
    // } else if (cargoInLowerBelts() ^ cargoInUpperBelts()) {
    //   setLowerOnlyMode();
    // } else if (/*no cargo, but gates are down TODO: make intake command that gives the gate status to cargo*/) {
    //   setIntakeMode();
    // } else if (/* cargo gates are up */) {
    //   setStopMode();
    // }

    // if (/* shoot button is pressed */) {
    //   setBothMode();
    // }

  }

  public void setLowerBeltPercentOutput(double percent) {
    LowerBelts.set(ControlMode.PercentOutput, percent);
  }

  public void setUpperBeltPercentOutput(double percent) {
    UpperBelts.set(ControlMode.PercentOutput, percent);
  }

  public void setLowerBeltRPM(double rpm) {
    LowerBelts.set(ControlMode.Velocity, rpm * 2048 / 600);
  }

  public void setUpperBeltRPM(double rpm) {
    UpperBelts.set(ControlMode.Velocity, rpm * 2048 / 600);
  }

  /**
   * enable Stop mode, conveyor motors are off
   */
  public void setStopMode(){
    mode = Mode.STOP;
  }

  /**
   * enable Intake mode, motors are on but no cargo is loaded
   */
  public void setIntakeMode(){
    mode = Mode.INTAKE;
  }

  /**
   * enable LowerOnly mode, one cargo loaded (in the top belt)
   */
  public void setLowerOnlyMode(){
    mode = Mode.LOWERONLY;
  }

  /**
   * enable UpperOnly mode - two cargo loaded
   */
  public void setUpperOnlyMode(){
    mode = Mode.UPPERONLY;
  }

  /**
   * enable Both mode - release one cargo from the conveyor belts
   */
  public void setBothMode(){
    mode = Mode.BOTH;
  }

  public void setReverseMode(){
    mode = Mode.REVERSE;
  }

  /** 
   * Stop all motors
   */
  public void stopIndexer() {
    mode = Mode.STOP;
    setLowerBeltPercentOutput(0.0);
    setUpperBeltPercentOutput(0.0);
  }

  /**
   * cargoInLowerBelts - monitor sensor 1 for a ball in the lower conveyor belt
   * 
   * @return true if a ball is in the lower conveyor belt
   */
  public boolean cargoInLowerBelts() {
    return ! lowerSensor.get();
  }

  /**
   * cargoInUpperBelts - monitor sensor 2 for a ball in the upper conveyor belt
   * 
   * @return true if a ball is in the upper conveyor belt
   */
  public boolean cargoInUpperBelts() {
    return ! upperSensor.get();
  }

  /**
   * stopLowerBelts() - stop the lower belts motor
   */
  private void stopLowerBelts() {
    setLowerBeltRPM(0);
  }

  /**
   * stopUpperBelts() - stop the upper belts motor
   */
  private void stopUpperBelts() {
    setUpperBeltPercentOutput(0);
  }

  public void coastMode() {
    LowerBelts.setNeutralMode(NeutralMode.Coast);
    UpperBelts.setNeutralMode(NeutralMode.Coast);
  }

}