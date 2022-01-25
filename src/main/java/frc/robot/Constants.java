// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Allocate CAN Ids from here. 
    // This avoids accidentally assigning the same CAN id to two different devices.
    public static final class canId {
      // CAN Id 0 is off limits. Typically unconfigured devices default to CAN id zero. This will 
      // create problems if you already have a device using CAN id 0 on the CAN bus.
      public static final int DoNotUse_canId0 = 0;

     // When assigning a CAN ID, rename the constant to something descriptive. Such as
     // when assigning CAN 1 rename "canId1" to "driveLeftLead" or "pigeonIMU"
     public static final int canId1_front_left_drive  = 1;
     public static final int canId2_front_right_drive = 2; 
     public static final int canId3_back_right_drive  = 3;
     public static final int canId4_back_left_drive   = 4;
     public static final int canId5_lower_belts       = 5;
     public static final int canId6_upper_belts       = 6;
     public static final int canId7_flywheel_left     = 7;
     public static final int canId8_flywheel_right    = 8;
     public static final int canId9 = 9;
     public static final int canId10 = 10;
     public static final int canId11_front_left_steer  = 11;
     public static final int canId12_front_right_steer = 12;
     public static final int canId13_back_right_steer  = 13;
     public static final int canId14_back_left_steer   = 14;
     public static final int canId15_pigeon_imu = 15;
     public static final int canId16 = 16;
     public static final int canId17 = 17;
     public static final int canId18_intake = 18;
     public static final int canId19 = 19;
     public static final int canId20 = 20;
     public static final int canId21_front_left_encoder  = 21;
     public static final int canId22_front_right_encoder = 22;
     public static final int canId23_back_left_encoder   = 23;
     public static final int canId24_back_right_encoder  = 24;
  }

  public static final class indexConstants {
    public static final int indexLowerBelts = canId.canId5_lower_belts;
    public static final int indexUpperBelts = canId.canId6_upper_belts;
  }

  public static final class digitalIOConstants {
    // assign digital IO (DIO) ports 0-9
    public static final int dio0_indexerSensor1 = 0;
    public static final int dio1_indexerSensor2 = 1;
    public static final int dio2_indexerSensor3 = 2;
  }

  public static final class currentLimits {
    public static SupplyCurrentLimitConfiguration m_currentlimitMain = new SupplyCurrentLimitConfiguration(true, 35, 1, 1);
    public static SupplyCurrentLimitConfiguration m_currentlimitSecondary = new SupplyCurrentLimitConfiguration(true, 25, 1, 1);
  }

}
