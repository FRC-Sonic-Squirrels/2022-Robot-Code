// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.canId;

public class ArmSubsystem extends SubsystemBase {
  //TODO: check if these are brushless motors 
  CANSparkMax m_armLeadMotor = new CANSparkMax(canId.CANID19_ARM_LEAD_MOTOR, MotorType.kBrushless);
  CANSparkMax m_armFollowMotor = new CANSparkMax(canId.CANID20_ARM_FOLLOW_MOTOR, MotorType.kBrushless);

  SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
  RelativeEncoder m_throughBoreEncoder;

  double targetAngle = 0.0;
  double ticksWhenStraightUp = 0;
  double rpm2degreesPerSecond = 60.0/360.0;
  double encoderTicks2degrees = 360.0/8192.0;

  SparkMaxPIDController m_armPID;
  
  //TODO: Find the actual channels
  DigitalInput limitSwitchFront = new DigitalInput(1);
  DigitalInput limitSwitchBack = new DigitalInput(2);
  //TODO: find true values 
  double m_maxEncoderValue = 2000;
  double m_minEncoderValue = -2000;

  public ArmSubsystem() {
    m_armLeadMotor.setIdleMode(IdleMode.kBrake);
    m_armFollowMotor.setIdleMode(IdleMode.kBrake);

    m_armFollowMotor.follow(m_armLeadMotor);

    m_throughBoreEncoder = m_armLeadMotor.getAlternateEncoder(kAltEncType, 8192);
    
    m_armPID = m_armLeadMotor.getPIDController();
    m_armPID.setFeedbackDevice(m_throughBoreEncoder);
    m_armPID.setOutputRange(-1, 1);
    //TODO: figure out how we use trapezoidal stuff 
    m_armPID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    m_armPID.setSmartMotionMaxAccel(0.5, 0);
    //TODO: velocity 45degress per second ish 
    m_armPID.setSmartMotionMaxVelocity(0.75, 0);
    //TODO: 1 degree error 22.75 ticks every degress 
    m_armPID.setSmartMotionAllowedClosedLoopError(0.05, 0);
    //good for preventing small changes but this can also be done with the joystick itself 
    //m_armPID.setSmartMotionMinOutputVelocity(0.05, 0);
    

    // TODO: not sure how setting the conversion interacts with PID control
    m_throughBoreEncoder.setPositionConversionFactor(360.0/8192.0);

    // TODO: need to figure out how to zero the arm position
    ticksWhenStraightUp = m_throughBoreEncoder.getPosition();
  }

  public double getAngleDegrees() {
    return m_throughBoreEncoder.getPosition();
  }

  //This would be encoder rotation values I think 
  public void setArmToSpecificRotation(double encoderValue){
    //makes sure it doesn't go over or under? Maybe use similar logic in periodic to stop or does the limit switch handle that? 
    if(encoderValue > m_maxEncoderValue) {encoderValue = m_maxEncoderValue;}
    else if(encoderValue < m_minEncoderValue) {encoderValue = m_minEncoderValue;}

    m_armPID.setReference(encoderValue, ControlType.kPosition);
  }

  public void setArmAngle(double angle) {
    targetAngle = angle;
  }

  //TODO: configure tolerance
  public boolean isAtAngle(){
    return (Math.abs(getAngleDegrees() - targetAngle) < 1.0);
  }

  public void holdAngle(double angle){
    //We need to be able to hold a angle for stage 2 of auto climbing. 
    //Setting the angle once and leaving it will lead to the arm slipping I think 
    // TODO: just set the setpoint
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
    //this maybe makes the arm stop?? 
    if(limitSwitchFront.get() || limitSwitchBack.get()){
      m_armPID.setReference(m_throughBoreEncoder.getPosition(), ControlType.kPosition);
    }

    SmartDashboard.putNumber("Arm Angle deg", getAngleDegrees());
    SmartDashboard.putNumber("Arm Vel (deg/s)", m_armLeadMotor.getEncoder().getVelocity()*rpm2degreesPerSecond);
    SmartDashboard.putNumber("Arm SetPoint", targetAngle);
    SmartDashboard.putNumber("Arm Error", targetAngle - getAngleDegrees());
    // SmartDashboard.putBoolean("Arm limit", );
    SmartDashboard.putNumber("Arm %output", m_armLeadMotor.getAppliedOutput());
    SmartDashboard.putNumber("Arm Current", m_armLeadMotor.getOutputCurrent());

  }
}
