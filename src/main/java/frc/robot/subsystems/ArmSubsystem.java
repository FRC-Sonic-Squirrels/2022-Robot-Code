// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.canId;

// Helpful link for converting between CTRE and Rev
// https://docs.revrobotics.com/sparkmax/software-resources/migrating-ctre-to-rev

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax m_armLeadMotor = new CANSparkMax(canId.CANID19_ARM_LEAD_MOTOR, MotorType.kBrushless);
  private CANSparkMax m_armFollowMotor = new CANSparkMax(canId.CANID20_ARM_FOLLOW_MOTOR, MotorType.kBrushless);

  private int kCPR = 8192;   // ticks per revolution
  private SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
  private RelativeEncoder m_throughBoreEncoder;

  private double m_targetAngle = 0.0;
  private double zeroedEncoderAngle = -20.4;
  private double rpm2degreesPerSecond = 60.0/360.0;
  private double degrees2rotations = 1.0/360.0;
  private double toleranceDegrees = 1.0;
  // arm angle = encoder angle * constant ratio
  private double m_encoderToArmRatio = 0.428571;
  
  private SparkMaxPIDController m_armPID;
  private double kP = 3.5;  // 4.0
  private double kI = 0.0001;
  private double kD = 0.0;
  private double kIz = 0.005;
  private double kFF = 0.0;
  private double kMaxOutput = 0.8;
  private double kMinOutput = -0.8;

  private double maxAngleDegree = 23.6;
  private double minAngleDegree = -20.5;

  private int m_numberOfTimesReinitalized = 0;

  private Robot m_robot;

  public ArmSubsystem(Robot robot) {

    SmartDashboard.putNumber("ARM number of reinitalize", m_numberOfTimesReinitalized);

    m_robot = robot;
    m_armLeadMotor.restoreFactoryDefaults();
    m_armFollowMotor.restoreFactoryDefaults();

    m_armLeadMotor.setIdleMode(IdleMode.kBrake);
    m_armFollowMotor.setIdleMode(IdleMode.kBrake);

    m_armFollowMotor.follow(m_armLeadMotor);

    m_throughBoreEncoder = m_armLeadMotor.getAlternateEncoder(kAltEncType, kCPR);
    
    m_armPID = m_armLeadMotor.getPIDController();
    m_armPID.setFeedbackDevice(m_throughBoreEncoder);
    m_armPID.setOutputRange(-1, 1);
    m_armPID.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 0);
    // Acceleration is in RPM/s. 45 degrees per second per second.
    m_armPID.setSmartMotionMaxAccel(60*(45.0/360.0), 0);
    // velocity is in RPM. 7.5 RPM is 45 degrees per second
    // for reference JVN calc claims max velocity of about 450 degrees per second.
    m_armPID.setSmartMotionMaxVelocity(60*(45.0/360.0), 0);
    // Error is in rotations
    m_armPID.setSmartMotionAllowedClosedLoopError(1/360.0, 0);
    //good for preventing small changes but this can also be done with the joystick itself 
    //m_armPID.setSmartMotionMinOutputVelocity(0.05, 0);
  
    // set PID coefficients
    m_armPID.setP(kP);
    m_armPID.setI(kI);
    m_armPID.setD(kD);
    m_armPID.setIZone(kIz);
    m_armPID.setFF(kFF);
    m_armPID.setOutputRange(kMinOutput, kMaxOutput);

    // Reduce CAN traffic when possible
    // https://www.hi-im.kim/canbus
    //MotorUtils.setSparkMaxStatusSlow(m_armFollowMotor);
    // we don't need fast updates of sensor velocity
    //m_armLeadMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 101);
    // we do need updates of sensor position
    //m_armLeadMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);

    // don't need frequent updates for follow motor
    m_armFollowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 396);
    m_armFollowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 401);

    m_throughBoreEncoder.setPositionConversionFactor(1.0);

    //because of gear box the encoder is spinning the wrong way
    m_throughBoreEncoder.setInverted(true);
  
    // Arm will start on a hard stop, part way back with a limit switch
    zeroEncoder();
    m_targetAngle = zeroedEncoderAngle;
  }

  private void initalizeMotors(){
    m_armLeadMotor.setIdleMode(IdleMode.kBrake);
    m_armFollowMotor.setIdleMode(IdleMode.kBrake);

    m_armFollowMotor.follow(m_armLeadMotor);

    m_throughBoreEncoder = m_armLeadMotor.getAlternateEncoder(kAltEncType, kCPR);
    
    m_armPID = m_armLeadMotor.getPIDController();
    m_armPID.setFeedbackDevice(m_throughBoreEncoder);
    m_armPID.setOutputRange(-1, 1);
    m_armPID.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 0);
    // Acceleration is in RPM/s. 45 degrees per second per second.
    m_armPID.setSmartMotionMaxAccel(60*(45.0/360.0), 0);
    // velocity is in RPM. 7.5 RPM is 45 degrees per second
    // for reference JVN calc claims max velocity of about 450 degrees per second.
    m_armPID.setSmartMotionMaxVelocity(60*(45.0/360.0), 0);
    // Error is in rotations
    m_armPID.setSmartMotionAllowedClosedLoopError(1/360.0, 0);
    //good for preventing small changes but this can also be done with the joystick itself 
    //m_armPID.setSmartMotionMinOutputVelocity(0.05, 0);
  
    // set PID coefficients
    m_armPID.setP(kP);
    m_armPID.setI(kI);
    m_armPID.setD(kD);
    m_armPID.setIZone(kIz);
    m_armPID.setFF(kFF);
    m_armPID.setOutputRange(kMinOutput, kMaxOutput);

    // Reduce CAN traffic when possible
    // https://www.hi-im.kim/canbus
    //MotorUtils.setSparkMaxStatusSlow(m_armFollowMotor);
    // we don't need fast updates of sensor velocity
    //m_armLeadMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 101);
    // we do need updates of sensor position
    //m_armLeadMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);

    // don't need frequent updates for follow motor
    m_armFollowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 396);
    m_armFollowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 401);

    m_throughBoreEncoder.setPositionConversionFactor(1.0);

    //because of gear box the encoder is spinning the wrong way
    m_throughBoreEncoder.setInverted(true);
  }

  /**
   * Hold - hold the arm in place using positional control
   */
  public void hold() {
    m_armPID.setReference(getEncoderValue(), ControlType.kPosition);
  }
  
  /**
   * coastMode - set the arm to coast mode
   */
  public void coastMode() {
    m_armLeadMotor.setIdleMode(IdleMode.kCoast);
    m_armFollowMotor.setIdleMode(IdleMode.kCoast);
  }
 
  /**
  *  brakeMode - set the arm to brake mode
  */
  public void brakeMode() {
    m_armLeadMotor.setIdleMode(IdleMode.kBrake);
    m_armFollowMotor.setIdleMode(IdleMode.kBrake);
  }

  /**
   * setArmAngle - sets the arm to a specific angle in degrees
   * 
   * @param angleDegrees arm angle in degrees
   */
  public void setArmAngle(double angleDegrees) {
    if(angleDegrees > maxAngleDegree) {
      angleDegrees = maxAngleDegree;
    }
    else if (angleDegrees < minAngleDegree) {
      angleDegrees = minAngleDegree;
    }
    m_targetAngle = angleDegrees;
    double encoderValue = angleToEncoderRotations(angleDegrees);
    m_armPID.setReference(encoderValue, ControlType.kPosition);
  }

  /**
   * convert from angle to encoder value
   */ 
  public double angleToEncoderRotations(double angleDegrees) {
    return (angleDegrees - zeroedEncoderAngle) * degrees2rotations / m_encoderToArmRatio;
  }

  public void setTolerance(double toleranceDegrees) {
    this.toleranceDegrees = toleranceDegrees;
  }

  public boolean isAtAngle(){
    return (Math.abs(getArmAngle() - m_targetAngle) <= toleranceDegrees);
  }

  public void setArmPercentOutput(double percentage){
    // set voltage to motor
    m_armPID.setReference(percentage * 11, ControlType.kVoltage);
  }

  public void zeroEncoder(){
    m_throughBoreEncoder.setPosition(0);
  }

  public double getEncoderValue(){
    return m_throughBoreEncoder.getPosition();
  }

  public double encoderRotationsToAngle(double encoderRotations) {
    return ((encoderRotations * m_encoderToArmRatio / degrees2rotations) + zeroedEncoderAngle);
  }

  public double getArmAngle() {
    return (m_throughBoreEncoder.getPosition() * m_encoderToArmRatio * 360) + zeroedEncoderAngle;
  }

  public void setMotorCoastMode(){
    m_armLeadMotor.setIdleMode(IdleMode.kCoast);
    m_armFollowMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setMotorBreakMode(){
    m_armLeadMotor.setIdleMode(IdleMode.kBrake);
    m_armFollowMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {

    if(m_robot.isDisabled()){
      double leadPidkP = m_armLeadMotor.getPIDController().getP();
      SmartDashboard.putString("ARM last error lead", m_armLeadMotor.getLastError().toString());
      SmartDashboard.putNumber("ARM kp value", leadPidkP);

      if(leadPidkP != kP){
        m_numberOfTimesReinitalized++;
        initalizeMotors();

        SmartDashboard.putNumber("ARM number of reinitalize", m_numberOfTimesReinitalized);
      }
      
    }

    SmartDashboard.putNumber("Arm Angle deg", getArmAngle());
    SmartDashboard.putNumber("Arm Vel (deg per sec)", m_armLeadMotor.getEncoder().getVelocity()*rpm2degreesPerSecond);
    SmartDashboard.putNumber("Arm SetPoint", m_targetAngle);
    SmartDashboard.putNumber("Arm Error", m_targetAngle - getArmAngle());
    SmartDashboard.putNumber("Arm %output", m_armLeadMotor.getAppliedOutput());
    SmartDashboard.putNumber("Arm Current", m_armLeadMotor.getOutputCurrent());
    // SmartDashboard.putNumber("Arm rpm To Degrees Per Second", rpm2degreesPerSecond);
    // SmartDashboard.putNumber("Arm tolerance Degrees", toleranceDegrees);
    // SmartDashboard.putNumber("Arm maximum Angle Degree", maxAngleDegree);
    // SmartDashboard.putNumber("Arm minimum Angle Degree", minAngleDegree);

  }
}
