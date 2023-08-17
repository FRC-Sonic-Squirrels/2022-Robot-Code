// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
//import edu.wpi.first.wpilibj.drive.Vector2d;
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
    // RoboRIO Can Ids

    // CAN Id 0 is off limits. Typically unconfigured devices default to CAN id zero. This will
    // create problems if you already have a device using CAN id 0 on the CAN bus.
    public static final int DoNotUse_canId0 = 0;

    // When assigning a CAN ID, rename the constant to something descriptive. Such as
    // when assigning CAN 1 rename "canId1" to "driveLeftLead" or "pigeonIMU"

    // Swerve module cadId assignments
    public static final int CANID1_FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int CANID11_FRONT_LEFT_MODULE_STEER_MOTOR = 11;
    public static final int CANID21_FRONT_LEFT_MODULE_STEER_ENCODER = 21;

    public static final int CANID2_FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2;
    public static final int CANID12_FRONT_RIGHT_MODULE_STEER_MOTOR = 12;
    public static final int CANID22_FRONT_RIGHT_MODULE_STEER_ENCODER = 22;

    public static final int CANID3_BACK_RIGHT_MODULE_DRIVE_MOTOR = 3;
    public static final int CANID13_BACK_RIGHT_MODULE_STEER_MOTOR = 13;
    public static final int CANID23_BACK_RIGHT_MODULE_STEER_ENCODER = 23;

    public static final int CANID4_BACK_LEFT_MODULE_DRIVE_MOTOR = 4;
    public static final int CANID14_BACK_LEFT_MODULE_STEER_MOTOR = 14;
    public static final int CANID24_BACK_LEFT_MODULE_STEER_ENCODER = 24;

    
    public static final int CANID8 = 8;
    public static final int CANID18_INTAKE = 18;
  }

  public static final class CANIVOR_canId {
    // CANIVOR Can Ids
    public static final String name = "CANivore";

    // CAN Id 0 is off limits. Typically unconfigured devices default to CAN id zero. This will
    // create problems if you already have a device using CAN id 0 on the CAN bus.
    public static final int DoNotUse_canId0 = 0;

    public static final int CANID5_LOWER_BELTS = 5;
    public static final int CANID6_UPPER_BELTS = 6;
    public static final int CANID15_pigeon_imu = 15;
    public static final int CANID16_flywheel_lead = 16;
    public static final int CANID17_flywheel_follow = 17;
    
  }


  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(320.2);
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(93.3);
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(282.4);
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(329.2);

  // Allocate Pneumatic channel constants from here
  // Prevents two pneumatic systems from sharing the same channel
  public static final class pneumatics {
    public static final int channel_0 = 0;
    public static final int channel_1 = 1;
    public static final int channel_2 = 2;
    public static final int channel_3 = 3;
    public static final int channel_4 = 4;
    public static final int channel_5 = 5;
    public static final int channel_6 = 6;
    public static final int channel_7 = 7;
    public static final int channel_8 = 8;
    public static final int channel_9 = 9;
    public static final int channel_10 = 10;
    public static final int channel_11 = 11;
    public static final int channel_12 = 12;
    public static final int channel_13 = 13;
    public static final int channel_14_intake = 14;
    public static final int channel_15_friction_brake = 15;  
  }
  public static final class digitalIOConstants {
    // assign digital IO (DIO) ports 0-9
    public static final int dio0_indexerSensor1 = 0;
    public static final int dio1_indexerSensor2 = 1;
    public static final int dio2_indexerSensor3 = 2;
  }

  public static final class currentLimits {
    public static SupplyCurrentLimitConfiguration m_currentlimit38A = new SupplyCurrentLimitConfiguration(true, 30, 38, 0.2);
    public static SupplyCurrentLimitConfiguration m_currentlimit30A = new SupplyCurrentLimitConfiguration(true, 25, 30, 0.2);
  }

  /**
   * The left-to-right distance between the drivetrain wheels
   *
   * Should be measured from center to center.
   */
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(25.0);
  /**
   * The front-to-back distance between the drivetrain wheels.
   *
   * Should be measured from center to center.
   */
  public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(24.0);

  // Set pigeon ID to -1 to disable and use NAVX on SPI.Port.kMXP
  public static final int DRIVETRAIN_PIGEON_ID = 15;

  public static class DriveFieldCentricConstant {
    public static final double TRANSLATION_MULTIPLIER = 0.8; 
    public static final double ROTATION_MULTIPLIER = 0.7; 
  }

  public static final class AutoConstants {
    public static final double maxVelocity = 2.5;     // meters per second
    public static final double maxAcceleration = 2.5; // meters per second per second

    // This kP worked for the DriveWithSetRotation command
    public static final double kPThetaController = 2.0; //1.5;
    public static final double kIThetaController = 0.0;
    public static final double kDThetaController = 0.0;

    // Feed Forward and PID values from SysId
    public static final double kP = 2.2941; // test bot = 2.3055;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kA = 0.435;  // SysId = 0.43516; // test bot = 0.12817;
    public static final double kV = 2.344;  // SysId = 2.344;   // test bot = 2.3423;
    public static final double kS = 0.628;  // SysID = 0.62811; // test bot = 0.53114;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            Drivetrain.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
  }

  public static class ShooterConstants {
    public static final double m_activated = 2000;
    public static final double IDLE = 500;

    public static final double MID_NODE_RPM = 4000;
    public static final double HIGH_NODE_RPM = 5000;

    public static final String ADJUSTABLE_OFFSET_RPM_STRING = "ADJUSTABLE RPM OFFSET";
  }
  // according to game manual field is 27 ft. (~823 cm) by 54 ft. (~1646 cm)

  public static class FieldConstants{


  } 

  public static class StartPoseConstants {
    public static Pose2d ORIGIN = new Pose2d(0.0, 0.0, new Rotation2d(0));
    }
}
