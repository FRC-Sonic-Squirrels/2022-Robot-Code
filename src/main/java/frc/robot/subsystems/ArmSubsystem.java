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
import edu.wpi.first.wpilibj.DigitalInput;
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
  private double ticksWhenStraightUp = 0;
  private double rpm2degreesPerSecond = 60.0/360.0;
  private double degrees2ticks = kCPR/360.0;
  private double toleranceDegrees = 1.0;
  // arm angle = encoder angle * constant ratio
  private double m_encoderToArmRatio = 0.428571;

  private SparkMaxPIDController m_armPID;
  
  //TODO: Find the actual channels
  private DigitalInput limitSwitchFront = new DigitalInput(1);
  private DigitalInput limitSwitchBack = new DigitalInput(2);

  //TODO: find true values 
  private double maxAngleDegree = 45;
  private double minAngleDegree = -45;

  private double m_armStepValue_testing = 0;


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
    m_armPID.setSmartMotionMaxVelocity(60*(45.0/360.0), 0);
    // Error is in rotations
    m_armPID.setSmartMotionAllowedClosedLoopError(1/360.0, 0);
    //good for preventing small changes but this can also be done with the joystick itself 
    //m_armPID.setSmartMotionMinOutputVelocity(0.05, 0);
    
    // TODO: maybe leave this alone and use raw encoder ticks
    m_throughBoreEncoder.setPositionConversionFactor(360.0/kCPR);

    // TODO: need to figure out how to zero the arm position.
    // maybe the arm will start on a hard stop, part way back with a limit switch
    ticksWhenStraightUp = m_throughBoreEncoder.getPosition();
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
    double encoderValue = ticksWhenStraightUp + angleToEncoderTicks(angleDegrees / m_encoderToArmRatio);
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
    return m_throughBoreEncoder.getPosition() * m_encoderToArmRatio;
  }

  public void setMotorCoastMode(){
    m_armLeadMotor.setIdleMode(IdleMode.kCoast);
    m_armFollowMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setMotorBreakMode(){
    m_armLeadMotor.setIdleMode(IdleMode.kBrake);
    m_armFollowMotor.setIdleMode(IdleMode.kBrake);
  }

  public void armStepBy(){
    m_armPID.setReference(m_throughBoreEncoder.getPosition() + m_armStepValue_testing, ControlType.kPosition);
  }

  public void updateTestingValues(){
    m_armStepValue_testing = SmartDashboard.getNumber("Arm Step Value", 0);
  }

  @Override
  public void periodic() {
    updateTestingValues();
    

    //this maybe makes the arm stop?? 
    if(limitSwitchFront.get() || limitSwitchBack.get()) {
      // TODO: getPosition will return an angle, need to convert back to ticks first
      // m_armPID.setReference(m_throughBoreEncoder.getPosition(), ControlType.kPosition);
    }

    SmartDashboard.putNumber("Arm_Subsystem Angle deg", getArmAngle());
    SmartDashboard.putNumber("Arm_Subsystem Vel (deg/s)", m_armLeadMotor.getEncoder().getVelocity()*rpm2degreesPerSecond);
    SmartDashboard.putNumber("Arm_Subsystem SetPoint", m_targetAngle);
    SmartDashboard.putNumber("Arm_Subsystem Error", m_targetAngle - getEncoderValue());
    // SmartDashboard.putBoolean("Arm limit", );
    SmartDashboard.putNumber("Arm_Subsystem %output", m_armLeadMotor.getAppliedOutput());
    SmartDashboard.putNumber("Arm_Subsystem Current", m_armLeadMotor.getOutputCurrent());
    SmartDashboard.putNumber("Arm_Subsystem kCPR", kCPR);
    SmartDashboard.putNumber("Arm_Subsystem target Angle", m_targetAngle);
    SmartDashboard.putNumber("Arm_Subsystem ticks When Strait Up", ticksWhenStraightUp);
    SmartDashboard.putNumber("Arm_Subsystem rpm To Degrees Per Second", rpm2degreesPerSecond);
    SmartDashboard.putNumber("Arm_Subsystem degrees To Ticks", degrees2ticks);
    SmartDashboard.putNumber("Arm_Subsystem tolerance Degrees", toleranceDegrees);
    SmartDashboard.putNumber("Arm_Subsystem encoder To Arm Ratio", m_encoderToArmRatio);
    SmartDashboard.putNumber("Arm_Subsystem maximum Angle Degree", maxAngleDegree);
    SmartDashboard.putNumber("Arm_Subsystem minimum Angle Degree", minAngleDegree);
    SmartDashboard.putNumber("Arm_Subsystem arm Step Value Testing", m_armStepValue_testing);
  }
}
