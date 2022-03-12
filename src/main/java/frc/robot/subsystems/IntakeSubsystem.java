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
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import static frc.robot.Constants.canId;

public class IntakeSubsystem extends SubsystemBase {

  enum Mode {
    STOP,
    FORWARD, 
    DYNAMIC,
    REVERSE
  };

  private WPI_TalonFX m_intake = new WPI_TalonFX(Constants.canId.CANID18_INTAKE);
  private TalonFXSensorCollection m_encoder;
  private Solenoid intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.pneumatics.channel_14_intake);
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

  private double m_forwardRpmValue = 3000;
  private double m_reverseRpmValue = -1000;

  private SupplyCurrentLimitConfiguration currentLimit =
    new SupplyCurrentLimitConfiguration(true, 20, 25, 0.1);

  public IntakeSubsystem(Drivetrain drivetrain) {
    
    m_drivetrain = drivetrain;
    m_intake.configFactoryDefault();
    m_intake.setInverted(false);
    m_intake.setNeutralMode(NeutralMode.Coast); 
    m_intake.configVoltageCompSaturation(10.0);
    m_intake.enableVoltageCompensation(true);
    m_intake.configSupplyCurrentLimit(currentLimit);

    m_intake.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);

    m_intake.config_kP(kPIDLoopIdx, 0.2);
    m_intake.config_kI(kPIDLoopIdx, 0.0);
    m_intake.config_kD(kPIDLoopIdx, 0.0);
    m_intake.config_kF(kPIDLoopIdx, 0.05);
    m_intake.config_IntegralZone(kPIDLoopIdx, 100);


    // Reduce CAN traffic a little, we
    m_intake.setStatusFramePeriod(StatusFrame.Status_1_General, 40);
    m_intake.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 40);

    m_encoder = m_intake.getSensorCollection();
    
    intakeRPM = 0.0;
  }
  // I think the motor, when retracted, is also paired with a stop command

  @Override
  public void periodic() {

    testingRpmValues();
    if(mode == Mode.STOP){
      setIntakePercentOutput(0);
    } else if(mode == Mode.FORWARD){
      setIntakeMotorRPM(m_forwardRpmValue);
    } else if(mode == Mode.DYNAMIC){
      setIntakeToSpeed();
    } else if(mode == Mode.REVERSE){
      setIntakeMotorRPM(m_reverseRpmValue);
    }

    SmartDashboard.putNumber("Intake_Subsystem RPM", m_encoder.getIntegratedSensorVelocity() * 600 / 2048);
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
    // SmartDashboard values have to match those in periodic() for values to update correctly
    m_forwardRpmValue = SmartDashboard.getNumber("Intake_Subsystem forward RPM Value", m_forwardRpmValue);
    m_reverseRpmValue = SmartDashboard.getNumber("Intake_Subsystem reverse RPM Value", m_reverseRpmValue);
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
   * extend piston to deploy intake
   */
  public void deployIntake() {
    intakeSolenoid.set(true);
    m_isDeployed = true;
  }

  /**
   * retract piston to retract intake
   */
  public void retractIntake() {
    intakeSolenoid.set(false);
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
    setIntakePercentOutput(0);
    retractIntake();
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