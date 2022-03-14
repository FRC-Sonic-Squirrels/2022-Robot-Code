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

    public static final int CANID7 = 7;
    public static final int CANID8 = 8;
    public static final int CANID18_INTAKE = 18;
    public static final int CANID19_ARM_LEAD_MOTOR = 19;
    public static final int CANID20_ARM_FOLLOW_MOTOR = 20;
  }

  public static final class CANIVOR_canId {
    // CANIVOR Can Ids
    public static final String name = "CANivore";

    // CAN Id 0 is off limits. Typically unconfigured devices default to CAN id zero. This will
    // create problems if you already have a device using CAN id 0 on the CAN bus.
    public static final int DoNotUse_canId0 = 0;

    public static final int CANID5_LOWER_BELTS = 5;
    public static final int CANID6_UPPER_BELTS = 6;
    public static final int CANID9_ELEVATOR_LEAD_TALON = 9;
    public static final int CANID10_ELEVATOR_FOLLOW_TALON = 10;
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
    public static final double TRANSLATION_MULTIPLIER = 0.825; 
    public static final double ROTATION_MULTIPLIER = 0.5; 
  }

  public static final class AutoConstants {
    public static final double maxVelocity = 2.5;     // meters per second
    public static final double maxAcceleration = 2.5; // meters per second per second

    // This kP worked for the DriveWithSetRotation command
    public static final double kPThetaController = 1.5;
    public static final double kIThetaController = 0.0;
    public static final double kDThetaController = 0.0;

    // Feed Forward and PID values from SysId
    public static final double kP = 2.2941; // test bot = 2.3055;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kA = 0.43516; // test bot = 0.12817;
    public static final double kV = 2.344;   // test bot = 2.3423;
    public static final double kS = 0.62811; // test bot = 0.53114;

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
    public static final double elevatorSpeedMultiplier = 1.0;
    public static final double ELEVATOR_MAX_HEIGHT = 22;
  }
  
  public static class ArmConstants{
    public static final double m_maxEncoderValue = 2000;
    public static final double m_minEncoderValue = -2000;

    public static final double CLIMBING_BACK_ANGLE = -13.0;
    public static final double CLIMBING_MIDDLE_ANGLE = -1;
    public static final double CLIMBING_FORWARD_ANGLE = 22;
    public static final double CLIMBING_NEXT_BAR_ANGLE = 15.6;
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
  //TODO: MAKE SURE THESE VALUES ARE CORRECT BEFORE WE TEST AUTONOMOUS 
  public static class AutoClimbConstants{
    public static class Stage_1 {
      public static final double ELEVATOR_PULL_HEIGHT = 0;
      public static final double ARM_TARGET_ANGLE = 0;
      public static final double ELEVATOR_SWITCH_TO_ARM_HEIGHT = 0;
  }
  public static class Stage_2 {
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

  public static class LimelightConstants{
    public static final double LIMELIGHT_HEIGHT_INCHES = 29.962;
    public static final double TARGET_HEIGHT_INCHES = 104.0;
    public static final double LIMELIGHT_PITCH_DEGREES = 40;
    public static final double HIGH_HUB_RADIUS_INCHES = 26.6875;

    //TODO: final distance between camera and the center of robot
    public static final Transform2d LIMELIGHT_TO_ROBOT = new Transform2d(new Translation2d(0.5, 0.2), new Rotation2d(0));
  }

  public static class VisionPipelineIndex {
    // TODO: find out what values the red and blue pipelines need to be
    public static final int RED_PIPELINE = 1;
    public static final int BLUE_PIPELINE = 2;
  }
  public static class FieldConstants{
    public static Translation2d BLUE_CARGO_1 = new Translation2d( Units.inchesToMeters(42), Units.inchesToMeters(44.4));
    public static Translation2d BLUE_CARGO_2 = new Translation2d( Units.inchesToMeters(198), Units.inchesToMeters(72));
    public static Translation2d BLUE_CARGO_3 = new Translation2d( Units.inchesToMeters(297.6), Units.inchesToMeters(7.2));
    public static Translation2d BLUE_CARGO_4 = new Translation2d( Units.inchesToMeters(412.8), Units.inchesToMeters(36));
    public static Translation2d BLUE_CARGO_5 = new Translation2d( Units.inchesToMeters(472.8), Units.inchesToMeters(198));
    public static Translation2d BLUE_CARGO_6 = new Translation2d( Units.inchesToMeters(290.4), Units.inchesToMeters(312));
    public static Translation2d BLUE_CARGO_7 = new Translation2d( Units.inchesToMeters(196.8), Units.inchesToMeters(246));
    public static Translation2d RED_CARGO_1 = new Translation2d( Units.inchesToMeters(605), Units.inchesToMeters(280));
    public static Translation2d RED_CARGO_2 = new Translation2d( Units.inchesToMeters(456), Units.inchesToMeters(252));
    public static Translation2d RED_CARGO_3 = new Translation2d( Units.inchesToMeters(350), Units.inchesToMeters(314));
    public static Translation2d RED_CARGO_4 = new Translation2d( Units.inchesToMeters(240), Units.inchesToMeters(290));
    public static Translation2d RED_CARGO_5 = new Translation2d( Units.inchesToMeters(176), Units.inchesToMeters(126));
    public static Translation2d RED_CARGO_6 = new Translation2d( Units.inchesToMeters(357), Units.inchesToMeters(12));
    public static Translation2d RED_CARGO_7 = new Translation2d( Units.inchesToMeters(456), Units.inchesToMeters(251));
    public static Translation2d HUB_CENTER = new Translation2d( Units.inchesToMeters(328), Units.inchesToMeters(162));
    public static Translation2d BLUE_LOW = new Translation2d( Units.inchesToMeters(130), Units.inchesToMeters(264));
    public static Translation2d BLUE_MID = new Translation2d( Units.inchesToMeters(88), Units.inchesToMeters(266));
    public static Translation2d BLUE_HIGH = new Translation2d( Units.inchesToMeters(62), Units.inchesToMeters(264));
    public static Translation2d BLUE_TRANSVERSAL = new Translation2d( Units.inchesToMeters(38), Units.inchesToMeters(264));
    public static Translation2d BLUE_PAD_1 = new Translation2d( Units.inchesToMeters(130), Units.inchesToMeters(216));
    public static Translation2d BLUE_PAD_2 = new Translation2d( Units.inchesToMeters(130), Units.inchesToMeters(312));
    public static Translation2d RED_LOW = new Translation2d( Units.inchesToMeters(520), Units.inchesToMeters(58));//change
    public static Translation2d RED_MID = new Translation2d( Units.inchesToMeters(562), Units.inchesToMeters(58));//change
    public static Translation2d RED_HIGH = new Translation2d( Units.inchesToMeters(586), Units.inchesToMeters(58));//change
    public static Translation2d RED_TRANSVERSAL = new Translation2d( Units.inchesToMeters(607), Units.inchesToMeters(58));//change
    public static Translation2d RED_PAD_1 = new Translation2d( Units.inchesToMeters(518), Units.inchesToMeters(108));
    public static Translation2d RED_PAD_2 = new Translation2d( Units.inchesToMeters(518), Units.inchesToMeters(12));

    public static Translation2d INFRONT_RED_CARGO_4 = new Translation2d( Units.inchesToMeters(240), Units.inchesToMeters(300));
    public static Translation2d BEHIND_RED_CARGO_4 = new Translation2d( Units.inchesToMeters(210), Units.inchesToMeters(290));

  } 

  public static class StartPoseConstants {
    public static Pose2d ORIGIN          = new Pose2d(0.0, 0.0, new Rotation2d(0));
    
    // TODO: make new starting poses right against the hub
    public static Pose2d BLUE_BOTTOM    = new Pose2d( Units.feetToMeters(26.8), Units.feetToMeters(5.64),   new Rotation2d(0.05) );
    public static Pose2d BLUE_MID_BOTTOM= new Pose2d( Units.feetToMeters(21.87), Units.feetToMeters(7.52),  new Rotation2d(-0.75) );
    public static Pose2d BLUE_MID_TOP   = new Pose2d( Units.feetToMeters(19.18), Units.feetToMeters(13.72), new Rotation2d(3*Math.PI/2+0.05) );
    public static Pose2d BLUE_TOP       = new Pose2d( Units.feetToMeters(21.01), Units.feetToMeters(18.48), new Rotation2d(Math.PI+0.75) );

    public static Pose2d RED_TOP        = new Pose2d( Units.feetToMeters(27.2), Units.feetToMeters(21.35),  new Rotation2d(0.05+Math.PI) );
    public static Pose2d RED_MID_TOP    = new Pose2d( Units.feetToMeters(32.13), Units.feetToMeters(19.48), new Rotation2d(-0.75+Math.PI) );
    public static Pose2d RED_MID_BOTTOM = new Pose2d( Units.feetToMeters(34.82), Units.feetToMeters(13.28), new Rotation2d(3*Math.PI/2+0.05+Math.PI) );
    public static Pose2d RED_BOTTOM     = new Pose2d( Units.feetToMeters(32.99), Units.feetToMeters(8.52),  new Rotation2d(Math.PI+0.75+Math.PI) );

    public static Pose2d BLUE_DEF_BOTTOM= new Pose2d( Units.feetToMeters(25.4), Units.feetToMeters(9.45), new Rotation2d(Math.PI-0.35));
    public static Pose2d BLUE_DEF_TOP   = new Pose2d( Units.feetToMeters(22.9), Units.feetToMeters(15),   new Rotation2d(Math.PI-0.35));
    public static Pose2d RED_DEF_TOP    = new Pose2d( Units.feetToMeters(28.6), Units.feetToMeters(17.55),new Rotation2d(-0.35));
    public static Pose2d RED_DEF_BOTTOM = new Pose2d( Units.feetToMeters(31.1), Units.feetToMeters(12),   new Rotation2d(3*Math.PI/2-0.35));
  }

}
