// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.subsystems.Drivetrain;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
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
     public static final int canId9_main_talon = 9;
     public static final int canId10_sub_talon = 10;
     public static final int canId11_friction_brake_solenoid  = 11;
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

  // Allocate Pneumatic channel constants from here
  // Prevents two pneumatic systems from sharing the same channel
  // public static final class Pneumatics {

  //   // TODO: check if pneumatic channel 0 can be used

  //   public static final int channel1_friction_brake_solenoid = 1;
  //   public static final int channel2 = 2;

  // }

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

  /**
   * The left-to-right distance between the drivetrain wheels
   *
   * Should be measured from center to center.
   */
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(19.0);
  /**
   * The front-to-back distance between the drivetrain wheels.
   *
   * Should be measured from center to center.
   */
  public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(19.0);

  // Set pigeon ID to -1 to disable and use NAVX on SPI.Port.kMXP
  public static final int DRIVETRAIN_PIGEON_ID = 15;

  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 11;
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 21;
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(142.4 + 180);

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2;
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 12;
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22;
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(65.4 + 180);

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 4;
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 14;
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 24;
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(28.1 + 180);

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 3;
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 13;
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 23;
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(182.7 - 180);

  public static final class AutoConstants {
    // This kP worked for the DriveWithSetRotation command
    public static final double kPThetaController = 3.0;
    public static final double kIThetaController = 0.0;
    public static final double kDThetaController = 0.02;

    // Feed Forward and PID values from SysId
    public static final double kP = 2.3055;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kA = 0.12817;
    public static final double kV = 2.3423;
    public static final double kS = 0.53114;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            Drivetrain.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
  }

  // according to game manual field is 27 ft. (~823 cm) by 54 ft. (~1646 cm)
  public static final Vector2d HUB_CENTER = new Vector2d(8.23, 4.11);
  public static final Pose2d ROBOT_1M_LEFT_OF_HUB =
      new Pose2d(HUB_CENTER.x - 1, HUB_CENTER.y, new Rotation2d(0));

      public static final class elevatorConstants {
        public static final int deploySolenoid1 = 0;
        public static final int deploySolenoid2 = 2;
        public static final int brakeSolenoid = 5;
        public static final int elevatorWinch = 12;
        public static final int elevatorPivotTimeout = 30;
        public static final int elevatorSlotIdx = 1;
    }

}
