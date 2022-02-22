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

    
    public static final int CANID5_LOWER_BELTS = 5;
    public static final int CANID6_UPPER_BELTS = 6;
    public static final int CANID7_FLYWHEEL = 7;
    public static final int CANID8_FRICTION_BRAKE_SOLENOID = 8;
    public static final int CANID9_ELEVATOR_LEAD_TALON = 9;
    public static final int CANID10_ELEVATOR_FOLLOW_TALON = 10;
    public static final int canId15_pigeon_imu = 15;
    public static final int canId16 = 16;
    public static final int canId17 = 17;
    public static final int CANID18_INTAKE = 18;
    public static final int CANID19_ARM_LEAD_MOTOR = 19;
    public static final int CANID20_ARM_FOLLOW_MOTOR = 20;
  }

  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(176.7);
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(28.0);
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(254.3);
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(145.9);

  // Allocate Pneumatic channel constants from here
  // Prevents two pneumatic systems from sharing the same channel
  public static final class pneumatics {
    public static final int channel_0_friction_brake_solenoid = 0;
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
    public static final int channel_14 = 14;
    public static final int channel_15 = 15;  
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

  //TODO: check actual gear ratio
  public static class IntakeConstants{
    public static final double gearRatio = 1;
  }

  public static class ElevatorConstants{
    public static final double elevatorSpeedMultiplier = 0.2;
  }
  
  public static class ArmConstants{
    public static final double m_maxEncoderValue = 2000;
    public static final double m_minEncoderValue = -2000;
  }

  public static class ShooterConstants {
    //TODO: find the actual activated and idle values
    public static final double m_activated = 2000;
    public static final double m_idle = 500;
  }
// according to game manual field is 27 ft. (~823 cm) by 54 ft. (~1646 cm)
public static class HubCentricConstants{
  public static final Vector2d HUB_CENTER = new Vector2d(8.23, 4.11);
  public static final Pose2d HUB_CENTER_POSE2D = new Pose2d(8.23, 4.11, new Rotation2d());
  public static final double FORWARD_MULTIPLIER = 0.5;
  public static final double SIDEWAYS_MULTIPLIER = 0.3;

}

public static final Pose2d ROBOT_1M_LEFT_OF_HUB =
    new Pose2d(HubCentricConstants.HUB_CENTER.x - 1,HubCentricConstants.HUB_CENTER.y , new Rotation2d(0));
  //TODO: MAKE SURE THESE VALUES ARE CORRECT BEFORE WE TEST AUTONOMUS 
  public static class AutoClimbConstants{
    public static class Stage_1{
      public static final double ELEVATOR_PULL_HEIGHT = 0;
      public static final double ARM_TARGET_ANGLE = 0;
      public static final double ELEVATOR_SWITCH_TO_ARM_HEIGHT = 0;
    }
    public static class Stage_2{
      //overshoot to let the elevator extend all the way then hold the correct angle
      public static final double ARM_STARTING_ANGLE = 0;
      public static final double ELEVATOR_EXTENSION_HEIGHT = 0;
      public static final double ARM_HOLD_ANGLE = 0;
      public static final double ELEVATOR_PULL_HEIGHT = 0;

      public static final double ELEVATOR_BRING_ARM_TO_OTHER_SIDE_HEIGHT = 0;
      public static final double ARM_BRING_AROUND_ANGLE = 0;

      public static final double ELEVATOR_PULL_TO_SWITCH_TO_ARM_HEIGHT = 0;

      public static final double ELEVATOR_LIFT_TO_SWITCH_TO_ARM_HEIGHT = 0;
    }
  }
  
  public static class VisionConstants{
    public static final double CAMERA_HEIGHT_INCHES = 42.0;
    public static final double TARGET_HEIGHT_INCHES = 104.0;
    public static final double CAMERA_PITCH_DEGREES = 34.0;

    //TODO: final distance between camera and the center of robot
    public static final Transform2d CAMERA_TO_ROBOT = new Transform2d(new Translation2d(0.5, 0.2), new Rotation2d());

  }

  public static class FieldConstants{
    public Translation2d BLUE_CARGO_1 = new Translation2d( Units.inchesToMeters(42), Units.inchesToMeters(44.4)); //this is blue alliance right?
    public Translation2d BLUE_CARGO_2 = new Translation2d( Units.inchesToMeters(198), Units.inchesToMeters(72));
    public Translation2d BLUE_CARGO_3 = new Translation2d( Units.inchesToMeters(297.6), Units.inchesToMeters(7.2));
    public Translation2d BLUE_CARGO_4 = new Translation2d( Units.inchesToMeters(412.8), Units.inchesToMeters(36));
    public Translation2d BLUE_CARGO_5 = new Translation2d( Units.inchesToMeters(472.8), Units.inchesToMeters(198));
    public Translation2d BLUE_CARGO_6 = new Translation2d( Units.inchesToMeters(290.4), Units.inchesToMeters(312));
    public Translation2d BLUE_CARGO_7 = new Translation2d( Units.inchesToMeters(196.8), Units.inchesToMeters(246));
    public Translation2d RED_CARGO_1 = new Translation2d( Units.inchesToMeters(605), Units.inchesToMeters(280));
    public Translation2d RED_CARGO_2 = new Translation2d( Units.inchesToMeters(257), Units.inchesToMeters(441));
    public Translation2d RED_CARGO_3 = new Translation2d( Units.inchesToMeters(350), Units.inchesToMeters(314));
    public Translation2d RED_CARGO_4 = new Translation2d( Units.inchesToMeters(235), Units.inchesToMeters(275));
    public Translation2d RED_CARGO_5 = new Translation2d( Units.inchesToMeters(174), Units.inchesToMeters(127));
    public Translation2d RED_CARGO_6 = new Translation2d( Units.inchesToMeters(357), Units.inchesToMeters(12));
    public Translation2d RED_CARGO_7 = new Translation2d( Units.inchesToMeters(460), Units.inchesToMeters(246));
    public Translation2d HUB_CENTER = new Translation2d( Units.inchesToMeters(324), Units.inchesToMeters(162));
    public Translation2d BLUE_LOW = new Translation2d( Units.inchesToMeters(130), Units.inchesToMeters(264));
    public Translation2d BLUE_MID = new Translation2d( Units.inchesToMeters(85), Units.inchesToMeters(264));
    public Translation2d BLUE_HIGH = new Translation2d( Units.inchesToMeters(62), Units.inchesToMeters(264));
    public Translation2d BLUE_TRANSVERSAL = new Translation2d( Units.inchesToMeters(38), Units.inchesToMeters(264));
    public Translation2d BLUE_PAD_1 = new Translation2d( Units.inchesToMeters(130), Units.inchesToMeters(216));
    public Translation2d BLUE_PAD_2 = new Translation2d( Units.inchesToMeters(130), Units.inchesToMeters(312));
    public Translation2d RED_LOW = new Translation2d( Units.inchesToMeters(518), Units.inchesToMeters(60));
    public Translation2d RED_MID = new Translation2d( Units.inchesToMeters(562), Units.inchesToMeters(60));
    public Translation2d RED_HIGH = new Translation2d( Units.inchesToMeters(586), Units.inchesToMeters(60));
    public Translation2d RED_TRANSVERSAL = new Translation2d( Units.inchesToMeters(607), Units.inchesToMeters(60));
    public Translation2d RED_PAD_1 = new Translation2d( Units.inchesToMeters(518), Units.inchesToMeters(108));
    public Translation2d RED_PAD_2 = new Translation2d( Units.inchesToMeters(518), Units.inchesToMeters(12));
  } 

  
}
