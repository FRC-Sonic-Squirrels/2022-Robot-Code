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

public class IndexerSubsystem extends SubsystemBase {

  enum Mode {
    INTAKE,
    EJECT,
    EJECTPAUSE,
    REVERSE,
    STOP
  };

  private WPI_TalonFX UpperBelts;
  private WPI_TalonFX LowerBelts;
  // TODO: add color sensor for cargo in upper belts
  private DigitalInput Sensor1 = new DigitalInput(digitalIOConstants.dio0_indexerSensor1);
  private DigitalInput Sensor2 = new DigitalInput(digitalIOConstants.dio1_indexerSensor2);
  private DigitalInput Sensor3 = new DigitalInput(digitalIOConstants.dio2_indexerSensor3);
  private boolean ballReady4IndexerLast = false;
  private boolean ballExitingLast = false;
  private boolean ejectBallStep1 = false;
  private boolean ejectBallStep2 = false;
  private boolean ejectBallStep3 = false;
  private int ballCount = 0;
  private Mode mode = Mode.STOP;

  public IndexerSubsystem() {

    LowerBelts = new WPI_TalonFX(canId.canId5_lower_belts);
    UpperBelts = new WPI_TalonFX(canId.canId6_upper_belts);

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
    boolean ballReady4Indexer = ballReadyForIndexer();
    boolean ballExiting = ballExiting();
    boolean ballStaged = ballStaged();

    //SmartDashboard.putNumber("Belt Amp", indexerBelts.getcurrent());
    //SmartDashboard.putNumber("ball count", ballCount);
    //SmartDashboard.putString("indexer state", mode.name());
    //SmartDashboard.putNumber("Belt RPM", LowerBelts.getSelectedSensorVelocity() * 600 / 2048);
    //SmartDashboard.putNumber("Kicker RPM", UpperBelts.getSelectedSensorVelocity() * 600 / 2048);


    // TODO: logic on running and stopping belts must completely change for 2022
    if (mode == Mode.STOP) {
      stopIndexer();
    }
    if (mode == Mode.EJECT) {
      setKickerPercentOutput(0.8);
      setBeltsPercentOutput(0.6);
    }
    if (mode == Mode.EJECTPAUSE) {
      setKickerPercentOutput(0.8);
      setBeltsPercentOutput(0.0);
    } 
    else if (mode == Mode.REVERSE) {
      setKickerPercentOutput(-0.5);
      setBeltsPercentOutput(-0.6);
    } 
    else if (mode == Mode.INTAKE) {
      // Normal, non-eject mode
      SmartDashboard.putNumber("Eject State", 0);

      if (ballExiting) {
        // ball exiting, but we're not shooting so stop the belts and kicker
        stopKicker();
        stopBelts();
        if (ballReady4Indexer) {
          // stop hopper
        }
        else {
          // secondary intake is empty, so keep running the hopper
        }
      }
      else if (ballReady4Indexer == false) {
        // no ball exiting
        // no ball staged
        // no ball in secondary intake, run hopper
        setBeltsPercentOutput(0.0);
      }
      else {
        // no ball exiting
        // no ball staged
        // ball in secondary intake, pull it into staged
        setBeltsPercentOutput(0.6);
      }
    }

    // increase ball count as balls enter the indexer
    if (ballReady4Indexer != ballReady4IndexerLast && ballReady4Indexer == false) {
      ballCount += 1;
    }
    ballReady4IndexerLast = ballReady4Indexer;

    // decrease ballCount as balls leave the indexer
    if (ballExiting != ballExitingLast && ballExiting == false) {
      ballCount -= 1;
    }
    ballExitingLast = ballExiting;
  }

  public void setBeltsPercentOutput(double percent) {
    LowerBelts.set(ControlMode.PercentOutput, percent);
  }

  public void setKickerPercentOutput(double percent) {
    UpperBelts.set(ControlMode.PercentOutput, percent);
  }

  public void setBeltsRPM(double rpm) {
    LowerBelts.set(ControlMode.Velocity, rpm * 2048 / 600);
  }

  public void setKickerRPM(double rpm) {
    UpperBelts.set(ControlMode.Velocity, rpm * 2048 / 600);
  }

  public void ejectOneBall() {

    if (mode == Mode.EJECT) {
      // we're already in ejectMode
      return;
    }

    /**
     * steps:
     * 1. run indexer until ball exiting (get ready to shoot)
     * 2. run indexer until ball not exiting (shooting!)
     * 3. stop indexer when ball ready to exit (ready for next shot)
     */
    mode = Mode.EJECT;
    ejectBallStep1 = true;
    ejectBallStep2 = ballExiting();
    ejectBallStep3 = false;

  }

  /**
   * enable Intake mode, pull balls into intake
   */
  public void setEjectPauseMode(){
    mode = Mode.EJECTPAUSE;
  }

  /**
   * enable Intake mode, pull balls into intake
   */
  public void setStopMode(){
    mode = Mode.STOP;
  }

  /**
   * enable Intake mode, pull balls into intake
   */
  public void setIntakeMode(){
    mode = Mode.INTAKE;
  }

  /**
   * enable Eject mode - eject balls for firing
   */
  public void setEjectMode(){
    mode = Mode.INTAKE;
  }

  /**
   * enable Reverse mode - reverse balls out of indexo through hopper
   */
  public void setReverseMode(){
    mode = Mode.REVERSE;
  }

  /** 
   * Stop all motors
   */
  public void stopIndexer() {
    mode = Mode.STOP;
    setBeltsPercentOutput(0.0);
    setKickerPercentOutput(0.0);
  }

  /**
   * ballReadyForIndexer - monitor sensor 1 for a ball ready to be indexed
   * 
   * @return true if a ball is waiting to be indexed
   */
  public boolean ballReadyForIndexer() {
    return ! Sensor1.get();
  }

  /**
   * ballStaged - monitor sensor 2 for a ball that is staged
   * 
   * @return true if a ball is staged
   */
  public boolean ballStaged() {
    return ! Sensor2.get();
  }

  /**
   * ballExiting - monitor sensor 3 for a ball that is at the kickers
   * 
   * @return true if a ball is at the kickers
   */
  public boolean ballExiting() {
    return ! Sensor3.get();
  }

  /**
   * stopBelts() - stop the belts motor
   */
  private void stopBelts() {
    setBeltsRPM(0);
  }

  /**
   * stopKicker() - stop the kicker motor
   */
  private void stopKicker() {
    setKickerPercentOutput(0);
  }

  /**
   * getBallCount() - return the number of balls in the indexer
   * 
   * @return int ball count
   */
  public int getBallCount() {
    return ballCount;
  }

  /**
   * setBallCount() - set the number of balls in the indexer
   */
  public void setBallCount(int BallCount) {
    ballCount = BallCount;
  }

}