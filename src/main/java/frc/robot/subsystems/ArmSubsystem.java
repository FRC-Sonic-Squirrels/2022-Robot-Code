// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.team2930.lib.util.MotorUtils;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private double zeroedEncoderAngle = -15.6;
  private double rpm2degreesPerSecond = 60.0/360.0;
  private double degrees2ticks = kCPR/360.0;
  private double toleranceDegrees = 1.0;
  // arm angle = encoder angle * constant ratio
  private double m_encoderToArmRatio = 0.428571;
  
  private SparkMaxPIDController m_armPID;
  private double kP = 0.1;
  private double kI = 0;
  private double kD = 0;
  private double kIz = 100;
  private double kFF = 0;
  private double kMaxOutput = 1;
  private double kMinOutput = -1;
  
  //TODO: Find the actual channels if physical limit switches are installed
  // private DigitalInput limitSwitchFront = new DigitalInput(5);
  // private DigitalInput limitSwitchBack = new DigitalInput(6);

  private double maxAngleDegree = 27.1;
  private double minAngleDegree = -15.6;

  private double m_armStepValueDegrees = 3.0;

  public ArmSubsystem() {
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
    MotorUtils.setSparkMaxStatusSlow(m_armFollowMotor);
    // we don't need fast updates of sensor velocity
    m_armLeadMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
    // we do need updates of sensor position
    m_armLeadMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);

    // TODO: maybe leave this alone and use raw encoder ticks
    m_throughBoreEncoder.setPositionConversionFactor(1.0);

    //because of gear box the encoder is spinning the wrong way
    m_throughBoreEncoder.setInverted(true);
  
    // Arm will start on a hard stop, part way back with a limit switch
    zeroEncoder();
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
    double encoderValue = angleToEncoderTicks((angleDegrees - zeroedEncoderAngle) / m_encoderToArmRatio);
    m_armPID.setReference(encoderValue, ControlType.kPosition);
  }

  /**
   * convert from angle to encoder value function
   */ 
  public double angleToEncoderTicks(double angleDegrees) {
    return angleDegrees * degrees2ticks;
  }

  public void setTolerance(double toleranceDegrees) {
    this.toleranceDegrees = toleranceDegrees;
  }

  public boolean isAtAngle(){
    return (Math.abs(getArmAngle() - m_targetAngle) <= toleranceDegrees);
  }

  public void setArmPercentOutput(double percentage){
    m_armLeadMotor.set(percentage);
  }

  public void zeroEncoder(){
    m_throughBoreEncoder.setPosition(0);
  }

  public double getEncoderValue(){
    return m_throughBoreEncoder.getPosition();
  }

  public double getArmAngle() {
    return (m_throughBoreEncoder.getPosition() * m_encoderToArmRatio * 360) - zeroedEncoderAngle;
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
    
    // TODO: we don't have limit switches on the robot yet
    //if(limitSwitchFront.get() || limitSwitchBack.get()) {
    //    TODO: getPosition will return an angle, need to convert back to ticks first
    //    m_armPID.setReference(m_throughBoreEncoder.getPosition(), ControlType.kPosition);
    //}

    SmartDashboard.putNumber("Arm Angle deg", getArmAngle());
    SmartDashboard.putNumber("Arm ticks", getEncoderValue());
    SmartDashboard.putNumber("Arm Vel (deg per sec)", m_armLeadMotor.getEncoder().getVelocity()*rpm2degreesPerSecond);
    SmartDashboard.putNumber("Arm SetPoint", m_targetAngle);
    SmartDashboard.putNumber("Arm Error", m_targetAngle - getEncoderValue());
    // SmartDashboard.putBoolean("Arm limit", );
    SmartDashboard.putNumber("Arm %output", m_armLeadMotor.getAppliedOutput());
    SmartDashboard.putNumber("Arm Current", m_armLeadMotor.getOutputCurrent());
    //SmartDashboard.putNumber("Armk CPR", kCPR);
    SmartDashboard.putNumber("Arm target Angle", m_targetAngle);
    //SmartDashboard.putNumber("Arm ticks When Strait Up", ticksWhenStraightUp);
    SmartDashboard.putNumber("Arm rpm To Degrees Per Second", rpm2degreesPerSecond);
    //SmartDashboard.putNumber("Arm degrees To Ticks", degrees2ticks);
    SmartDashboard.putNumber("Arm tolerance Degrees", toleranceDegrees);
    //SmartDashboard.putNumber("Arm encoder To Arm Ratio", m_encoderToArmRatio);
    //SmartDashboard.putNumber("Arm maximum Angle Degree", maxAngleDegree);
    //SmartDashboard.putNumber("Arm minimum Angle Degree", minAngleDegree);

  }
}
