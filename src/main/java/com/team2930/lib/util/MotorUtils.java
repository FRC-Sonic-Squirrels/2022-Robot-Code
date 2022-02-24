// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2930.lib.util;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

/**
 * Utilities for lowering (or raising) CAN update rates to reduce CAN traffic. To be used on motor
 * controllers that are follow motors, or do not do any velocity or positional control.
 * 
 * Thanks to:
 * https://github.com/4512OtterChaos/frc2020/blob/119f72d0c05b82cc3073ff0d15ac819d76450d67/src/main/java/frc/robot/common/OCConfig.java#L185-L205
 * 
 */
public class MotorUtils {

  /**
   * setCtreStatusSlow() - reduce the update rate for the CTRE motor controllers to reduce CAN traffic
   * 
   * Use this for follow motors and motors that do not need to report velocity or position.
   * 
   * @param motors This can be any of the CTRE motor classes like WPI_TalonFX, etc
   */
  public static void setCtreStatusSlow(BaseMotorController... motors) {
    for (BaseMotorController motor : motors) {
      motor.setStatusFramePeriod(StatusFrame.Status_1_General, 500);
      motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);
    }
  }

  public static void setFalconStatusFast(BaseMotorController... motors) {
    for (BaseMotorController  motor : motors) {
      motor.setStatusFramePeriod(StatusFrame.Status_1_General, 5);
      motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
    }
  }

  public static void setCtreStatusNormal(BaseMotorController... motors) {
    for (BaseMotorController motor : motors) {
      // not sure what the normal defaults are
      motor.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
      motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
    }
  }
  /**
   * setSparkMaxStatusSlow() - reduce the update rate for the REV motor controllers to reduce CAN traffic
   * 
   * Use this for follow motors and motors that do not need to report velocity or position.
   * 
   * @param motors This can be any of the REV motor classes like CANSparkMax, etc
   */
  public static void setSparkMaxStatusSlow(CANSparkMax... motors) {
    for (CANSparkMax motor : motors) {
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    }
  }

  public static void setSparkMaxStatusFast(CANSparkMax... motors) {
    for (CANSparkMax motor : motors) {
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
    }
  }

  public static void setSparkMaxStatusNormal(CANSparkMax... motors) {
    for (CANSparkMax motor : motors) {
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    }
  }

}
