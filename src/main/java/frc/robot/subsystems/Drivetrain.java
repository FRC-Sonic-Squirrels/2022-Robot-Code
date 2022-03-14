// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d; 
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.canId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.Vector2d;
import static frc.robot.Constants.*;

public class Drivetrain extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * 
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial
   * testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;

  // TODO: Measure the drivetrain's maximum velocity or calculate the theoretical.
  // The formula for calculating the theoretical maximum velocity is:
  // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  // By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  // An example of this constant for a Mk4 L2 module with Falcon 500s to drive is:
  // 6380.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
  // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND =
      6380.0 / 60.0 * SdsModuleConfigurations.MK4I_L2.getDriveReduction()
          * SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

  /**
   * The maximum acceleration of the robot.
   * 
   * Setting it to the same as MAX_VELOCITY means we expect to be able to accelerate to max speed in
   * one second.
   */
  public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED =
      MAX_VELOCITY_METERS_PER_SECOND;

  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a
  // measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
      MAX_VELOCITY_METERS_PER_SECOND
          / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  /**
   * The maximum rotational acceleration of the robot.
   * 
   * Setting it to the same as MAX_ANGULAR_VELOCITY means we expect to be able to accelerate to max
   * rotational speed in one second.
   */
  public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED =
      MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Front right
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back left
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back right
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

  private final SwerveDriveOdometry m_odometry;
  
  private final Field2d m_field = new Field2d();

  private WPI_Pigeon2 m_pigeon = new WPI_Pigeon2(Constants.CANIVOR_canId.CANID15_pigeon_imu, CANIVOR_canId.name);

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private SwerveModuleState[] m_desiredStates;

  private SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(AutoConstants.kS, AutoConstants.kV, AutoConstants.kA);

  private boolean isOdometrySet = false;

  /**
   * Object constructor
   */
  public Drivetrain() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    //TODO: Explore this, this looks like a good solution for validating the position through the limelight also could just be a better odometry system 
    // private final SwerveDrivePoseEstimator m_poseEstimator; 
    // m_poseEstimator = new SwerveDrivePoseEstimator(gyroAngle, initialPoseMeters, kinematics, stateStdDevs, localMeasurementStdDevs, visionMeasurementStdDevs);
    // m_poseEstimator = new SwerveDrivePoseEstimator(gyroAngle, initialPoseMeters, kinematics, stateStdDevs, localMeasurementStdDevs, visionMeasurementStdDevs, nominalDtSeconds);

    // m_poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
      
    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk4iSwerveModuleHelper.createFalcon500(...)
    // Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.
    //
    // By default we will use Falcon 500s in standard configuration. But if you use a different
    // configuration or motors you MUST change it. If you do not, your code will crash on startup.

    Mk4ModuleConfiguration swerveConfig = new Mk4ModuleConfiguration();
    swerveConfig.setDriveCurrentLimit(40);
    swerveConfig.setNominalVoltage(11.5);

    m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
        // This parameter is optional, but will allow you to see the current state of the module on
        // the dashboard.
        // tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
        // set current limit and nominal voltage
        swerveConfig,
        // This can either be STANDARD or FAST depending on your gear configuration
        Mk4iSwerveModuleHelper.GearRatio.L2,
        // This is the ID of the drive motor
        canId.CANID1_FRONT_LEFT_MODULE_DRIVE_MOTOR,
        // This is the ID of the steer motor
        canId.CANID11_FRONT_LEFT_MODULE_STEER_MOTOR,
        // This is the ID of the steer encoder
        canId.CANID21_FRONT_LEFT_MODULE_STEER_ENCODER,
        // This is how much the steer encoder is offset from true zero (In our case, zero is facing
        // straight forward)
        FRONT_LEFT_MODULE_STEER_OFFSET);

    // We will do the same for the other modules
    m_frontRightModule =
        Mk4iSwerveModuleHelper.createFalcon500(
            //tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2,0),
            swerveConfig,
            Mk4iSwerveModuleHelper.GearRatio.L2,
            canId.CANID2_FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            canId.CANID12_FRONT_RIGHT_MODULE_STEER_MOTOR,
            canId.CANID22_FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET);

    m_backLeftModule =
        Mk4iSwerveModuleHelper.createFalcon500(
            //tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4,0),
            swerveConfig,
            Mk4iSwerveModuleHelper.GearRatio.L2,
            canId.CANID4_BACK_LEFT_MODULE_DRIVE_MOTOR,
            canId.CANID14_BACK_LEFT_MODULE_STEER_MOTOR,
            canId.CANID24_BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET);

    m_backRightModule =
        Mk4iSwerveModuleHelper.createFalcon500(
            //tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6,0),
            swerveConfig,
            Mk4iSwerveModuleHelper.GearRatio.L2,
            canId.CANID3_BACK_RIGHT_MODULE_DRIVE_MOTOR,
            canId.CANID13_BACK_RIGHT_MODULE_STEER_MOTOR,
            canId.CANID23_BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET);

    m_odometry = new SwerveDriveOdometry(m_kinematics, getIMURotation());

    // TODO: set starting point on the field accurately
    m_odometry.resetPosition(new Pose2d(0.0, 0.0, new Rotation2d(0.0)), getIMURotation());

    m_desiredStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 0.0));

    tab.addNumber("Gyroscope Angle", () -> getIMURotation().getDegrees());
    tab.addNumber("Pose X", () -> m_odometry.getPoseMeters().getX());
    tab.addNumber("Pose Y", () -> m_odometry.getPoseMeters().getY());

    //SmartDashboard.putData("Field", m_field);
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently
   * facing to the 'forwards' direction.
   */
  public void zeroGyroscope() {
    setGyroscopeHeadingDegrees(0.0);

    m_odometry.resetPosition(
        new Pose2d(m_odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0)),
        Rotation2d.fromDegrees(0.0));
  }

  public void setGyroscopeHeadingDegrees(double deg) {
    m_pigeon.setYaw(deg);
    m_pigeon.setAccumZAngle(deg);
  }

  public void resetFieldCentric(){
    Pose2d currentPose = getPose();

    m_odometry.resetPosition(
      new Pose2d(currentPose.getX(),currentPose.getY(), new Rotation2d(0)), getIMURotation());
  }

  public void setGyroscopeHeadingRadians(double rad) {
    setGyroscopeHeadingDegrees(Math.toDegrees(rad));
  }

  /**
   * Set the current odometry pose
   * 
   * @param pose
   * @param rotation
   */
  public void setPose(Pose2d pose, Rotation2d rotation) {
    
    isOdometrySet = true; 
    
    m_odometry.resetPosition(pose, rotation);
  }

  public boolean isOdometrySet(){
    return isOdometrySet;
  }

  /**
   * get current angle from gyroscope, return Rotation2d object.
   * ONLY USE THIS IF YOU KNOW WHAT YOU ARE DOING 
   * FOR MOST CASES USE getRotation() instead
   * 
   * @return gyro angle in Rotation2d
   */
  public Rotation2d getIMURotation() {
      // return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
      return Rotation2d.fromDegrees(m_pigeon.getYaw());
  }

  public Rotation2d getRotation(){
    return m_odometry.getPoseMeters().getRotation();
  }


  public Rotation2d getGyroscopeRotationVelocity() {
    double[] xyz_dps = new double[3];
    m_pigeon.getRawGyro(xyz_dps);
    return Rotation2d.fromDegrees(xyz_dps[2]);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public double getVelocity(){
    SwerveModuleState m_actualStates[] = new SwerveModuleState[4];
    SwerveModuleState frontLeft = new SwerveModuleState(m_frontLeftModule.getDriveVelocity(), new Rotation2d(m_frontLeftModule.getSteerAngle()));
    SwerveModuleState frontRight = new SwerveModuleState(m_frontRightModule.getDriveVelocity(), new Rotation2d(m_frontRightModule.getSteerAngle()));
    SwerveModuleState backLeft = new SwerveModuleState(m_backLeftModule.getDriveVelocity(), new Rotation2d(m_backLeftModule.getSteerAngle()));
    SwerveModuleState backRight = new SwerveModuleState(m_backRightModule.getDriveVelocity(), new Rotation2d(m_backRightModule.getSteerAngle()));
    //new array with size 4, fill array with new module sates in the order : fl, fr, bl, br
    m_actualStates[0] = frontLeft;
    m_actualStates[1] = frontRight;
    m_actualStates[2] = backLeft;
    m_actualStates[3] = backRight;
    ChassisSpeeds cs = m_kinematics.toChassisSpeeds(m_actualStates);
    Vector2d vector =  new Vector2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
      
    return vector.magnitude(); 
  }
  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    if (desiredStates != null) {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_PER_SECOND); 

      m_frontLeftModule.set(velocityToDriveVolts(
          desiredStates[0].speedMetersPerSecond),
          desiredStates[0].angle.getRadians());
      m_frontRightModule.set(velocityToDriveVolts(
          desiredStates[1].speedMetersPerSecond),
          desiredStates[1].angle.getRadians());
      m_backLeftModule.set(velocityToDriveVolts(
          desiredStates[2].speedMetersPerSecond),
          desiredStates[2].angle.getRadians());
      m_backRightModule.set(velocityToDriveVolts(
          desiredStates[3].speedMetersPerSecond),
          desiredStates[3].angle.getRadians());
    }
    else {
      DriverStation.reportError("Null swerve state in setModulesStates()", false);
    }
  }

  /**
   * Convert target velocity to motor volts.
   * 
   * @param speedMetersPerSecond
   * @return volts
   */
  private double velocityToDriveVolts(double speedMetersPerSecond) {
    double ff = m_feedForward.calculate(speedMetersPerSecond);
    return MathUtil.clamp(ff, -MAX_VOLTAGE, MAX_VOLTAGE);

    // below is a naive guess for output volts, linear map of desired velocity to volts
    // return speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE;
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, getIMURotation());
  }

  /**
   * set desired swerve module states from chassisSpeed
   * 
   * @param chassisSpeeds
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    m_desiredStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
  }

  /**
   * Return the SwerveDriveKinematics for the drivetrain. Used for Trajectory calculation.
   * 
   * @return SwerveDriveKinematics
   */
  public SwerveDriveKinematics kinematics() {
    return m_kinematics;
  }

  public Rotation2d getWheelRotation() {
    double avgRotation = 0;
    for (int i = 0; i < 4; i++) {
      SwerveModuleState state = m_desiredStates[i];
      avgRotation += state.angle.getRadians();
    }

    return new Rotation2d(avgRotation / 4);
  }

  @Override
  public void periodic() {
    m_odometry.update(getIMURotation(),
        new SwerveModuleState(m_frontLeftModule.getDriveVelocity(),
            new Rotation2d(m_frontLeftModule.getSteerAngle())),
        new SwerveModuleState(m_frontRightModule.getDriveVelocity(),
            new Rotation2d(m_frontRightModule.getSteerAngle())),
        new SwerveModuleState(m_backLeftModule.getDriveVelocity(),
            new Rotation2d(m_backLeftModule.getSteerAngle())),
        new SwerveModuleState(m_backRightModule.getDriveVelocity(),
            new Rotation2d(m_backRightModule.getSteerAngle())));

    // TODO: how can we be sure m_desiredStates form Autonomous gets set before running this periodic()
    // and if not, will that mean we get occasional 20ms delays and/or duplicate states?
    setModuleStates(m_desiredStates);

    // Update pose in field simulation
    //m_field.setRobotPose(m_odometry.getPoseMeters());

    //SmartDashboard.putNumber("Drivetrain IMU Yaw", m_pigeon.getYaw());
    //SmartDashboard.putNumber("Drivetrain IMU Roll", m_pigeon.getRoll());
    //SmartDashboard.putNumber("Drivetrain IMU Pitch", m_pigeon.getPitch());
    //SmartDashboard.putNumber("Drivetrain odometry angle", m_odometry.getPoseMeters().getRotation().getDegrees());
  }
}
