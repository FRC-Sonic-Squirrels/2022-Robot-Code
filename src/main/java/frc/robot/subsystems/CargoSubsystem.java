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
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIVOR_canId;
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
    REVERSE,
    SHOOT,
    SHOOT_STEP2,
    SHOOT_PREP,
    IDLE
  };

  private WPI_TalonFX UpperBelts;
  private WPI_TalonFX LowerBelts;
  private DigitalInput lowerSensor = new DigitalInput(digitalIOConstants.dio0_indexerSensor1);
  private DigitalInput upperSensor = new DigitalInput(digitalIOConstants.dio1_indexerSensor2);
  private Mode mode = Mode.STOP;
  private double m_idleTime = 0;
  private boolean ejectOpponentCargo = true;
  private ColorSensorIntake colorSensorIntake;

  // TODO: find the real percent outputs of the conveyor belts
  private double m_lowerOutput = 0.8;
  private double m_upperOutput = 0.9;
  private long ejectTime = 0;

  public CargoSubsystem(ColorSensorIntake colorSensorIntake) {
    this.colorSensorIntake = colorSensorIntake;
    
    LowerBelts = new WPI_TalonFX(CANIVOR_canId.CANID5_LOWER_BELTS, CANIVOR_canId.name);
    UpperBelts = new WPI_TalonFX(CANIVOR_canId.CANID6_UPPER_BELTS, CANIVOR_canId.name);

    LowerBelts.configFactoryDefault();
    UpperBelts.configFactoryDefault();

    // reduce CAN traffic for motors (not using speed control)
    LowerBelts.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
    LowerBelts.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 199);
    UpperBelts.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
    UpperBelts.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 197);

    // Voltage limits, percent output is scaled to this new max
    LowerBelts.configVoltageCompSaturation(10);
    LowerBelts.enableVoltageCompensation(true);
    UpperBelts.configVoltageCompSaturation(10);
    UpperBelts.enableVoltageCompensation(true);

    // current limits, Max 20A
    LowerBelts.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 25, 0.1));
    UpperBelts.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 25, 0.1));

    // ramp rate, how long to take to get to full power
    LowerBelts.configOpenloopRamp(0.2);
    UpperBelts.configOpenloopRamp(0.1);

    // Brake mode
    LowerBelts.setNeutralMode(NeutralMode.Brake);
    UpperBelts.setNeutralMode(NeutralMode.Brake);

    // Invert
    LowerBelts.setInverted(false);
    UpperBelts.setInverted(false);

    LowerBelts.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    UpperBelts.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);


    // TODO: configure PID for lower and upper belts
    // Config PID values to control RPM
    // LowerBelts.config_kP(0, 0.15, 10);
    // LowerBelts.config_kI(0, 0.0, 10);
    // LowerBelts.config_kD(0, 1.5, 10);
    // LowerBelts.config_kF(0, 0.048, 10);

    // UpperBelts.config_kP(0, 0.15, 10);
    // UpperBelts.config_kI(0, 0.0, 10);
    // UpperBelts.config_kD(0, 1.5, 10);
    // UpperBelts.config_kF(0, 0.053, 10);

  }

  @Override
  public void periodic() {

    if (mode == Mode.STOP) {
      stopIndexer();
    } else if (mode == Mode.INTAKE) {
      if (cargoInUpperBelts()) {
        stopUpperBelts();
      }
      else {
        setUpperBeltPercentOutput(0.5);
      }

      if ((cargoInLowerBelts() && cargoInUpperBelts())
          || (ejectTime != 0)
          || (ejectOpponentCargo && colorSensorIntake.opponentCargoDetected())) {

        if (ejectOpponentCargo && colorSensorIntake.opponentCargoDetected()) {
          ejectTime = System.currentTimeMillis();
        }

        if (System.currentTimeMillis() - ejectTime < 200) {
          // eject opponent cargo mode
          setLowerBeltPercentOutput(-0.2);
        }
        else {
          ejectTime = 0;
          stopLowerBelts();
        }
      } else {
        ejectTime = 0;
        setLowerBeltPercentOutput(0.9);
      }

    } else if (mode == Mode.LOWERONLY) {
      stopUpperBelts();
      setLowerBeltPercentOutput(m_lowerOutput);
    } else if (mode == Mode.UPPERONLY) {
      stopLowerBelts();
      setUpperBeltPercentOutput(m_upperOutput);
    } else if (mode == Mode.BOTH) {
      // Normal, non-eject mode
      setUpperBeltPercentOutput(m_upperOutput);
      setLowerBeltPercentOutput(m_lowerOutput);
    } else if (mode == Mode.SHOOT) {
      if (!cargoInUpperBelts()) {
        setUpperBeltPercentOutput(0.9);
        setLowerBeltPercentOutput(0.6);
        mode = Mode.SHOOT_STEP2;
      } else {
        setUpperBeltPercentOutput(-0.5);
        setLowerBeltPercentOutput(-0.2);
      }
    } else if (mode == Mode.SHOOT_STEP2) {
      setUpperBeltPercentOutput(0.9);
      setLowerBeltPercentOutput(0.6);
    } else if(mode == Mode.SHOOT_PREP){
      if(!cargoInUpperBelts()){
        setStopMode();
        setUpperBeltPercentOutput(0);
        setLowerBeltPercentOutput(0);
      } else {
        setUpperBeltPercentOutput(-0.5);
        setLowerBeltPercentOutput(-0.2);
      }
    } else if(mode == Mode.IDLE){
      setUpperBeltPercentOutput(0);
      if(m_idleTime == 0){
        m_idleTime = System.currentTimeMillis();
      } else if(System.currentTimeMillis() - m_idleTime > 300){
        setShootPrepMode();
        m_idleTime = 0;
        setLowerBeltPercentOutput(0);
      }
      setLowerBeltPercentOutput(0.9);
    }else if (mode == Mode.REVERSE) {
      setUpperBeltPercentOutput(-m_lowerOutput);
      setLowerBeltPercentOutput(-m_upperOutput);
    } else {
      System.out.println("Indexer: unknown mode");
    }

    SmartDashboard.putString("Cargo: Mode", mode.name());
    SmartDashboard.putBoolean("Cargo in Upper", cargoInUpperBelts());
    SmartDashboard.putBoolean("Cargo in Lower", cargoInLowerBelts());
    // if(this.getCurrentCommand() != null){
    //   SmartDashboard.putString("AAA cargosubsystem current command", this.getCurrentCommand().toString());
    // } else {
    //   SmartDashboard.putString("AAA Cargosubsystem current command", "null");
    // }
    // SmartDashboard.putNumber("Cargo Upper Voltage", UpperBelts.getMotorOutputVoltage());
    // SmartDashboard.putNumber("Cargo Lower Voltage", LowerBelts.getMotorOutputVoltage());

    // TODO: convert to RPM
    // SmartDashboard.putNumber("Cargo RPM Upper", UpperBelts.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("Cargo RPM Lower", LoweBelts.getSelectedSensorVelocity());

  }

  public void updateTestingValues(){
    m_lowerOutput = SmartDashboard.getNumber("cargo subsystem lower motor speed", m_lowerOutput);
    m_upperOutput = SmartDashboard.getNumber("cargo subsystem upper motor speed", m_upperOutput);
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

  public void setBothBeltsPercentOutput(double percent) {
    LowerBelts.set(ControlMode.PercentOutput, percent);
    UpperBelts.set(ControlMode.PercentOutput, percent);
  }

  public void setEjectOpponentCargo (boolean eject) {
    ejectOpponentCargo = eject;
  }

  public boolean getEjectOpponentCargo (){
    return ejectOpponentCargo;
  }

  /**
   * enable Stop mode, conveyor motors are off
   */
  public void setStopMode() {
    ejectTime = 0;
    mode = Mode.STOP;
  }

  /**
   * enable Intake mode, motors are on but no cargo is loaded
   */
  public void setIntakeMode(){
    ejectTime = 0;
    mode = Mode.INTAKE;
  }

  /**
   * enable LowerOnly mode, one cargo loaded (in the top belt)
   */
  public void setLowerOnlyMode(){
    ejectTime = 0;
    mode = Mode.LOWERONLY;
  }

  /**
   * enable UpperOnly mode - two cargo loaded
   */
  public void setUpperOnlyMode(){
    ejectTime = 0;
    mode = Mode.UPPERONLY;
  }

  /**
   * enable Both mode - release one cargo from the conveyor belts
   */
  public void setBothMode(){
    ejectTime = 0;
    mode = Mode.BOTH;
  }

  public void setShootMode(){
    ejectTime = 0;
    mode = Mode.SHOOT;
  }

  public void setReverseMode(){
    ejectTime = 0;
    mode = Mode.REVERSE;
  }

  public void setShootPrepMode(){
    ejectTime = 0;
    mode = Mode.SHOOT_PREP;
  }

  public void setIdleMode(){
    ejectTime = 0;
    mode = Mode.IDLE;
  }

  /** 
   * Stop all motors
   */
  public void stopIndexer() {
    ejectTime = 0;
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