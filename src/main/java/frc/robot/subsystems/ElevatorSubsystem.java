// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.team2930.lib.util.MotorUtils;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIVOR_canId;


// Details on the TalonFX motion profile control can be found here:
// https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html
// Example code:
// https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20Talon%20FX%20(Falcon%20500)/PositionClosedLoop_AuxFeedForward/src/main/java/frc/robot/Robot.java


public class ElevatorSubsystem extends SubsystemBase {

  private WPI_TalonFX winch_lead_talon = new WPI_TalonFX(CANIVOR_canId.CANID9_ELEVATOR_LEAD_TALON, CANIVOR_canId.name);
  private WPI_TalonFX winch_follow_talon = new WPI_TalonFX(CANIVOR_canId.CANID10_ELEVATOR_FOLLOW_TALON, CANIVOR_canId.name);
  private Solenoid frictionBrakeSolenoid =
      new Solenoid(PneumaticsModuleType.REVPH, Constants.pneumatics.channel_15_friction_brake);
  private final double gearRatio = 0.08229; // 0.074;
  //TODO: note diameter is set to 1.9 on JVN 
  private final double winchDiameter_inches = 1.43;   // 1.25 diameter + string windings
  private final double winchCircumference = Math.PI * winchDiameter_inches;
  private final double maxExtensionInches = 26; //actually 24 letting more for unwinding
  private double heightSetpointInches = 0.0;
  private double toleranceInches = 0.05; 
  private double feedForwardClimbing = 0.025734; // from JVM calculator
  private double feedForwardDescending = 0.0257; //0.001;
  private final double ticks2distance = gearRatio * winchCircumference / 2048; 
  private boolean zeroed = false;

  public boolean m_atMaxheight;

  private double m_currentHeight;

  // the encoder increase as the elevator moves down, so invert sign of height vs ticks
  private double sensor_invert = -1.0;
  
  public ElevatorSubsystem() {
    winch_lead_talon.configFactoryDefault();
    winch_follow_talon.configFactoryDefault();
    
    TalonFXConfiguration leadConfig = new TalonFXConfiguration();

    leadConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

    // Details on elevator motors, gearing and calculated kP and kFF are here
    // https://docs.google.com/spreadsheets/d/1sOS_vM87iaKPZUFSJTqKqaFTxIl3Jj5OEwBgRxc-QGM/edit?usp=sharing
    // this also has suggest trapezoidal velocity profile constants.
    leadConfig.slot0.kF = 0.054; 
		leadConfig.slot0.kP = 0.48; //0.054836;
		leadConfig.slot0.kI = 0.0;
		leadConfig.slot0.kD = 0.0;
		leadConfig.slot0.integralZone = 0.0;
		leadConfig.slot0.closedLoopPeakOutput = 1.0;

    //do we need this if the command is updating the motion magic constraints? 
    //maybe have a safe default?  
    leadConfig.motionAcceleration = 30000; //60941;    //  20521 ticks/100ms     = 11 in/s
		leadConfig.motionCruiseVelocity = 15235;  //  20521 ticks/100ms/sec = 11 in/s^2

    leadConfig.slot0.allowableClosedloopError = toleranceInches / ticks2distance;

    // set config
    winch_lead_talon.configAllSettings(leadConfig);

    //use pid from slot 0 for motion magic 
    winch_lead_talon.selectProfileSlot(0, 0);

    winch_lead_talon.setNeutralMode(NeutralMode.Brake);
    winch_follow_talon.setNeutralMode(NeutralMode.Brake);

    // config hard limit switch for full down position
    winch_lead_talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        LimitSwitchNormal.NormallyOpen, 0);

    winch_follow_talon.follow(winch_lead_talon);
    winch_lead_talon.setInverted(false);
    winch_follow_talon.setInverted(false);

    // JVN calculator predicts 41.2 A per motor under load
    //TODO: Check new JVN prediction
    SupplyCurrentLimitConfiguration currentLimit =
        new SupplyCurrentLimitConfiguration(true, 20, 25, 0.1);
    winch_lead_talon.configSupplyCurrentLimit(currentLimit);
    winch_follow_talon.configSupplyCurrentLimit(currentLimit);

    winch_lead_talon.configOpenloopRamp(0.1);

    // Reduce CAN traffic where possible
    // https://docs.ctre-phoenix.com/en/latest/ch18_CommonAPI.html
    MotorUtils.setCtreStatusSlow(winch_follow_talon);
    // leave lead motor with default CAN settings. We need position and limit switch updates
    
    // NOTE: when we power up, we expect the elevator to be full down, triggering the lower limit switch.
    // if not, we need to move the elevator down to the lower limit switch (VERY SLOWLY).
    // hitting either limit switch must stop the elevator.

    // Stagger update frames to reduce congestion
    winch_follow_talon.setStatusFramePeriod(StatusFrame.Status_1_General, 47);
    winch_follow_talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 201);

    brakeOn();
    zeroHeight();
 
    // // set soft limit on forward movement (down)
    // winch_lead_talon.configForwardSoftLimitThreshold(heightToTicks(0.0));
    // winch_lead_talon.configForwardSoftLimitEnable(true);

    // set soft limit on reverse movement (Up)
    winch_lead_talon.configReverseSoftLimitThreshold(heightToTicks(maxExtensionInches));
    winch_lead_talon.configReverseSoftLimitEnable(true);
  }


  /**
   * 
   * @param acceleration accel in inches per second^2
   * @param cruiseVelocity max velocity in inches per second 
   */
  public void setMotionMagicConstraints(double cruiseVelocity, double desiredTimeToSpeed){
    //math adapted from howdybots jvn calculator equation
    double veloInTicks = cruiseVelocity * (12.15/winchCircumference) * 2048 / 10;
    double accelInTicks = veloInTicks / desiredTimeToSpeed;

    winch_lead_talon.configMotionAcceleration(accelInTicks);
    winch_lead_talon.configMotionCruiseVelocity(veloInTicks);


    //temporary for debugging 
    SmartDashboard.putNumber("Elevator MM Constraint desiredTimeToSpeed", desiredTimeToSpeed);
    SmartDashboard.putNumber("Elevator MM Constraint velo INCHES", cruiseVelocity);

    SmartDashboard.putNumber("Elevator MM Constraint accel TICKS", accelInTicks);
    SmartDashboard.putNumber("Elevator MM Constraint velo TICKS", veloInTicks);
  }

  public void setMotionMagicSetPoint(double heightInches) {
    // if (heightInches < 0.0) {
    //   heightInches = 0.0;
    // }
    if (heightInches > maxExtensionInches) {
      heightInches = maxExtensionInches;
    }
    
    winch_lead_talon.set(ControlMode.MotionMagic, heightToTicks(heightInches));

    // if (heightInches <= heightSetpointInches) {
    //   // lifting up robot, use more feed forward
    //   winch_lead_talon.set(TalonFXControlMode.MotionMagic, heightToTicks(heightInches),
    //       DemandType.ArbitraryFeedForward, feedForwardClimbing);
    // } else {
    //   // lowering robot, use less feed forward
    //   winch_lead_talon.set(TalonFXControlMode.MotionMagic, heightToTicks(heightInches),
    //       DemandType.ArbitraryFeedForward, feedForwardDescending);
    // }

    heightSetpointInches = heightInches;

    SmartDashboard.putNumber("Elevator MM Height SetPoint", heightInches);
  }

  public void setElevatorHeight(double heightInches) {
    if (heightInches < 0.0) {
      heightInches = 0.0;
    }
    if (heightInches > maxExtensionInches) {
      heightInches = maxExtensionInches;
    }

    if (heightInches <= heightSetpointInches) {
      // lifting up robot, use more feed forward
      winch_lead_talon.set(TalonFXControlMode.Position, heightToTicks(heightInches),
          DemandType.ArbitraryFeedForward, feedForwardClimbing);
    } else {
      // lowering robot, use less feed forward
      winch_lead_talon.set(TalonFXControlMode.Position, heightToTicks(heightInches),
          DemandType.ArbitraryFeedForward, feedForwardDescending);
    }
    // NOTE: this is how without arbitrary feed forward
    // winch_lead_talon.set(TalonFXControlMode.Position, heightToTicks(heightInches));

    heightSetpointInches = heightInches;
  }


  public double heightToTicks(double heightInches) {
    return sensor_invert * heightInches / ticks2distance;
  }

  public double ticksToHeight(double ticks) {
    return sensor_invert * ticks * ticks2distance;
  }

  /**
   * hold() - hold the elevator at the current height with PID
   */
  public void hold() {
    winch_lead_talon.set(TalonFXControlMode.Position, winch_lead_talon.getSelectedSensorPosition(),
        DemandType.ArbitraryFeedForward, feedForwardClimbing);
  }

  /**
   * @return true if withing tolerance of  target height
   */
  public boolean isAtHeight(double heightInches) {
    return (Math.abs(heightInches - getHeightInches()) < toleranceInches);
  }

  /**
   * isAtHeight() check if the elevator is at the target height.
   * 
   * @return true if the elevator is at the height setpoint
   */
  public boolean isAtHeight() {
    return  isAtHeight(heightSetpointInches);
  }

  /**
   * getHeightInches() returns the current height of the elevator in inches.
   * 
   * @return height of the elevator in inches
   */
  public double getHeightInches() {
    return  (getHeightTicks() * ticks2distance);
  }

  public double getHeightTicks() {
    return sensor_invert * winch_lead_talon.getSelectedSensorPosition();
  }

  /**
   * zeroHeight() resets the starting position of the elevator to the current height.
   *
   * This is called when the elevator triggers the lower limit switch. This needs to be done by a
   * command, that runs the elevator to the lower limit switch. VERY SLOWLY.
   */
  public void zeroHeight() {
    winch_lead_talon.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }

  /**
   * Manually run elevator motors. USE WITH CAUTION.
   */
  public void setWinchPercentOutput(double percent) {
    winch_lead_talon.set(ControlMode.PercentOutput, percent);
  }

  /**
   * stop() stops the elevator motors and sets the brake.
   */
  public void stop() {
    setWinchPercentOutput(0.0);
    brakeOn();
  }

  /**
   * brakeOn() turns on the brake.
   */
  public void brakeOff(){
    frictionBrakeSolenoid.set(true);
  }

  /**
   * brakeOff() turns off the brake.
   */
  public void brakeOn(){
    frictionBrakeSolenoid.set(false);
  }

  /**
   * atLowerLimit() returns true if the lower limit switch is triggered.
   */
  public boolean atLowerLimit() {
    return (1 == winch_lead_talon.isFwdLimitSwitchClosed());
  }

  @Override
  public void periodic() {
    // check if we triggered lower limit switch, and reset elevator to zero
    if(atLowerLimit()) {
      if (! zeroed) {
        // only zero height once per time hitting limit switch
        zeroHeight();
        zeroed = true;
      }
    }
    else {
      // not currently on limit switch, zero again next time we hit limit switch
      zeroed = false;
    }

    double currentHeight = getHeightInches();
    if(currentHeight >= maxExtensionInches){
      m_atMaxheight = true;
    } else {
      m_atMaxheight = false;
    }

    SmartDashboard.putBoolean("ELEVATOR AT THE MAX HEIGHT", m_atMaxheight);

    // if(this.getCurrentCommand() != null){
    //   SmartDashboard.putString("AAA elevator current command", this.getCurrentCommand().toString());
    // } else {
    //   SmartDashboard.putString("AAA elevator current command", "null");
    // }

    
    SmartDashboard.putNumber("Elevator height ticks", getHeightTicks());
    SmartDashboard.putNumber("Elevator Height (inches)", getHeightInches());
    SmartDashboard.putNumber("Elevator Height Set Point", heightSetpointInches);
    //SmartDashboard.putNumber("Elevator Height (ticks)", getHeightTicks());
    SmartDashboard.putNumber("Elevator current Vel (inches per s)", ticks2distance * 10.0 * winch_lead_talon.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Elevator current vel ticks", winch_lead_talon.getSelectedSensorVelocity() * 10.0);
    //SmartDashboard.putNumber("Elevator SetPoint inches", heightSetpointInches);
    //SmartDashboard.putNumber("Elevator SetPoint (ticks)", heightToTicks(heightSetpointInches));
    SmartDashboard.putNumber("Elevator Error", heightSetpointInches - getHeightInches());
    SmartDashboard.putBoolean("Elevator limit", atLowerLimit());
    SmartDashboard.putNumber("Elevator %output", winch_lead_talon.getMotorOutputPercent());
    SmartDashboard.putNumber("Elevator Current Lead", winch_lead_talon.getSupplyCurrent());
    SmartDashboard.putNumber("Elevator Current Follow", winch_follow_talon.getSupplyCurrent());
    SmartDashboard.putBoolean("Elevator Brake On", !frictionBrakeSolenoid.get());

    //debug values for MM. These should match the values from setMotionMagicConstraints()
    //note: These along with all the other config options are viewable and editable from phoenix tuner 
    SmartDashboard.putNumber("Elevator MM config acceleration", (winch_lead_talon.configGetParameter(ParamEnum.eMotMag_Accel, 0) * ticks2distance * 10));
    SmartDashboard.putNumber("Elevator MM config velocity ", (winch_lead_talon.configGetParameter(ParamEnum.eMotMag_VelCruise, 0) * ticks2distance * 10));
    SmartDashboard.putNumber("Elevator MM config S-curve", winch_lead_talon.configGetParameter(ParamEnum.eMotMag_SCurveLevel, 0));

    SmartDashboard.updateValues();

  }
}
