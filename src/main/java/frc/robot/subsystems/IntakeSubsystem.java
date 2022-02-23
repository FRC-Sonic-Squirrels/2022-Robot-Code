/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Relay;
import static frc.robot.Constants.canId;

public class IntakeSubsystem extends SubsystemBase {

  enum Mode {
    STOP,
    FORWARD, 
    DYNAMIC,
    REVERSE
  };

  private WPI_TalonFX m_intake = new WPI_TalonFX(canId.CANID18_INTAKE);
  private TalonFXSensorCollection m_encoder;
  private Relay intakeRelay = new Relay(0);
  private Drivetrain m_drivetrain;
  
  private double circOfIntake_meters = (1.4725 * Math.PI) * 0.0254;
  private double minIntakeRPM = 2500;
  private double maxIntakeRPM = 6000;
  private double intakeRPM = 0.0;
  private double m_desiredRPM = 0.0;
  private boolean m_isDeployed = false;
  private static int kPIDLoopIdx = 0;
  private static int kTimeoutMs = 30;
  private Mode mode = Mode.STOP;
  private double m_forwardRpmValue = 0;
  private double m_reverseRpmValue = 0;

  private SupplyCurrentLimitConfiguration currentLimit =
    new SupplyCurrentLimitConfiguration(true, 25, 30, 0.5);

  public IntakeSubsystem(Drivetrain drivetrain) {
    
    m_drivetrain = drivetrain;
    m_intake.configFactoryDefault();
    m_intake.setInverted(true);
    m_intake.setNeutralMode(NeutralMode.Coast); 
    m_intake.configVoltageCompSaturation(10.0);
    m_intake.configSupplyCurrentLimit(currentLimit);

    m_intake.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);

    m_intake.config_kP(kPIDLoopIdx, 0.2);
    m_intake.config_kI(kPIDLoopIdx, 0.0005);
    m_intake.config_kD(kPIDLoopIdx, 0);
    m_intake.config_kF(kPIDLoopIdx, 0.05);
    m_intake.config_IntegralZone(kPIDLoopIdx, 100);

    m_encoder = m_intake.getSensorCollection();
    //deployIntake();
    
    intakeRPM = 0.0;
    
    
  }
  // I think the motor, when retracted, is also paired with a stop command

  @Override
  public void periodic() {
    // boolean dynamic = SmartDashboard.getBoolean("Dynamic Mode", dynamicMode);
    // dynamicMode = dynamic;
    // if (dynamicMode) {
    //   setIntakeToSpeed();
    // } else {
    //   double ir = SmartDashboard.getNumber("Set Intake Motor RPM", 0.0);
    //   if (ir != intakeRPM) {
    //     intakeRPM = ir;
    //     setIntakeMotorRPM(intakeRPM);
    //   }
    //}
    testingRpmValues();
    if(mode == Mode.STOP){
      m_intake.setVoltage(0);
    } else if(mode == Mode.FORWARD){
      setIntakeMotorRPM(m_forwardRpmValue); //TODO: set RPM to actual value needed
    } else if(mode == Mode.DYNAMIC){
      setIntakeToSpeed();
    } else if(mode == Mode.REVERSE){
      setIntakeMotorRPM(-m_reverseRpmValue); //TODO: what does the speed have to be for reverse?
    }

    SmartDashboard.putNumber("Intake_Subsystem RPM", - m_encoder.getIntegratedSensorVelocity() * 600 / 2048);
    SmartDashboard.putNumber("Intake_Subsystem desired Motor RPM", m_desiredRPM);
    SmartDashboard.putNumber("Intake_Subsystem Robot Speed m per s", m_drivetrain.getVelocity());
    SmartDashboard.putNumber("Intake_Subsystem minimum Intake RPM", minIntakeRPM);
    SmartDashboard.putNumber("Intake_Subsystem maximum Intake RPM", maxIntakeRPM);
    SmartDashboard.putBoolean("Intake_Subsystem is Deployed", m_isDeployed);
    SmartDashboard.putNumber("Intake_Subsystem circumference Of Intake", circOfIntake_meters);
    SmartDashboard.putNumber("Intake_Subsystem forward RPM Value", m_forwardRpmValue);
    SmartDashboard.putNumber("Intake_Subsystem reverse RPM Value", m_reverseRpmValue);
    //SmartDashboard.putNumber("Robot Speed m per s", (m_drive.getLeftVelocity() + m_drive.getRightVelocity()) / 2.0);
  }
  
  public void testingRpmValues(){
    m_forwardRpmValue = SmartDashboard.getNumber("forward intake subsystem motor speed", 0);
    m_reverseRpmValue = SmartDashboard.getNumber("reverse intake subsystem motor speed", 0);
  }

  /**
   * Sets Intake Percent output to designated Percent
   */
  public void setIntakePercentOutput(double percent) {
    m_intake.set(ControlMode.PercentOutput, percent);
  }

  /**
   * Sets Intake RPM to designated RPM
   */
  public void setIntakeMotorRPM(double desiredRPM) {
    intakeRPM = desiredRPM;
    m_desiredRPM = desiredRPM;
    m_intake.set(ControlMode.Velocity, desiredRPM * 2048 / 600.0);
  }

  /**
   * Takes the speed at which the Robot moves and makes the Intake move a relative
   * speed
   */
  public void setIntakeToSpeed() {
    double robotMetersPerSec = m_drivetrain.getVelocity(); //check if m/s 
    double intakeRotationsPerSec = robotMetersPerSec / circOfIntake_meters;
    //Going Twice as Fast as the Robot Speed
    double _intakeRPM = intakeRotationsPerSec * 60 * 4.0;
    double desiredMotorRPM = _intakeRPM * Constants.IntakeConstants.gearRatio;
    if (desiredMotorRPM < minIntakeRPM) {
      desiredMotorRPM = minIntakeRPM;
    } else if (desiredMotorRPM > maxIntakeRPM) {
      desiredMotorRPM = maxIntakeRPM;
    }

    setIntakeMotorRPM(desiredMotorRPM);
    
  }

  /**
   * returns whether or not the intake motor is at the required speed
   */
  public boolean intakeAtDesiredRPM() {
    return (intakeRPM == m_desiredRPM);
  }

  /**
   * release and deploy the intake
   */
  public void deployIntake() {
    intakeRelay.set(Relay.Value.kReverse);
    m_isDeployed = true;
  }

  /**
   * reset solenoids
   */
  public void retractIntake() {
    // There is no retract on this robot. Just reset solenoids
    intakeRelay.set(Relay.Value.kForward);
    m_isDeployed = false;
  }

  public void toggleIntake() {
    if (!m_isDeployed) {
      deployIntake();
    }
    else {
      retractIntake();
    }
  }

  /**
   * Coasts the Intake to zero using new PID
   */
  public void coastToZero() {
    setStopMode();
    setIntakePercentOutput(0);
  }


  public void stop() {
    setStopMode();
    m_intake.setVoltage(0.0);
    setIntakeMotorRPM(0.0);
    intakeRelay.set(Relay.Value.kForward);
  }

  public boolean isDeployed(){
    return m_isDeployed;
  }  

  public void setStopMode(){
    mode = Mode.STOP;
  }

  public void setForwardMode(){
    mode = Mode.FORWARD;
  }

  public void setDynamicMode(){
    mode = Mode.DYNAMIC;
  }

  public void setReverseMode(){
    mode = Mode.REVERSE;
  }

}