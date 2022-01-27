// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class HubCentricCommand extends CommandBase {
  private Drivetrain m_drivetrain;
  private SwerveDriveKinematics m_kinematics;

  private Supplier<Double> m_sidewaysSupplier;
  private Supplier<Double> m_forwardSupplier;

  // copied values from Swerve Template Odometry
  private PIDController rotationalController = new PIDController(3.0, 0.0, 0.02);

  private Vector2d m_hubCenter = Constants.HUB_CENTER;
  
  public HubCentricCommand(Drivetrain drivetrain, Supplier<Double> sidewaysSupplier, Supplier<Double> forwardSupplier) {
    m_sidewaysSupplier = sidewaysSupplier;
    m_forwardSupplier = forwardSupplier;
    m_drivetrain = drivetrain;
    m_kinematics = m_drivetrain.kinematics();

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    rotationalController.enableContinuousInput(-Math.PI, Math.PI);
    rotationalController.setTolerance(Math.PI/180); //1 degree of wiggle room
  }

  @Override
  public void execute() {
    Rotation2d currentHeading = m_drivetrain.getGyroscopeRotation();
    Pose2d robotPosition = m_drivetrain.getPose();
    Vector2d robotVector = new Vector2d(robotPosition.getX(), robotPosition.getY());

    Rotation2d targetHeading = getTargetHeading(robotVector, m_hubCenter);
    double radius = Math.sqrt(Math.pow(m_hubCenter.x - robotPosition.getX(), 2) + Math.pow(m_hubCenter.y - robotPosition.getY(), 2));

    double rotationCorrection = rotationalController.calculate(currentHeading.getRadians(), targetHeading.getRadians());

    // TODO: Check if controller needs to be multiplied by max velocity
    // TODO: figure out how to move forward and back, sideways is a scaling factor 
    //       not a directional factor thus both strafeX and Y need to use sidewaysSupplier
    //       this means robot cant move forwards/change radius
    // 
    //       Possible solution: use robot centric to generate swerve module states for moving forward 
    //       average both states (arc strafe & forward movement) to get a forward motion and a arc? 
    //       as for rn robot should be able to maintain heading towards center and translate in a arc successfully 
    //       but not change radius/move forward & back. 

    var arcChassisSpeeds = new ChassisSpeeds();
    var forwardCHassisSpeeds = new ChassisSpeeds();

    if(m_forwardSupplier.get() != 0.0){
      // forwardCHassisSpeeds.vxMetersPerSecond = m_forwardSupplier.get() * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND;

      // forward/reverse is just orthogonal to tangent
      double orthogonalHeading = targetHeading.getRadians() - (Math.PI / 4.0);

      forwardCHassisSpeeds.vxMetersPerSecond = findStrafeX(1.0, orthogonalHeading, Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, m_forwardSupplier.get(), 0.5);
      forwardCHassisSpeeds.vyMetersPerSecond = findStrafeY(1.0, orthogonalHeading, Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, m_forwardSupplier.get(), 0.5);

    }
    if(m_sidewaysSupplier.get() != 0.0){
      double strafeX = findStrafeX(radius, targetHeading.getRadians(), Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, m_sidewaysSupplier.get(), 0.3);
      double strafeY = findStrafeY(radius, targetHeading.getRadians(), Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, m_sidewaysSupplier.get(), 0.3);

      arcChassisSpeeds.vxMetersPerSecond = strafeY;
      arcChassisSpeeds.vyMetersPerSecond = strafeX;
    }
    
    ChassisSpeeds finalChassisSpeeds = averageChassisSpeeds(arcChassisSpeeds, forwardCHassisSpeeds);

    m_drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(finalChassisSpeeds.vxMetersPerSecond
    , finalChassisSpeeds.vyMetersPerSecond, rotationCorrection, currentHeading));

    //TODO: check if need to flip order of coordinates from x,y to y,x
    SmartDashboard.putNumber("currentHeading", currentHeading.getDegrees());
    SmartDashboard.putNumber("targetHeading", targetHeading.getDegrees());
    SmartDashboard.putNumberArray("robotPosition", new double[] {robotPosition.getX(), robotPosition.getY()});
    
    SmartDashboard.putNumber("rotationCorrection", rotationCorrection);
    SmartDashboard.putNumberArray("strafe values", new double[] {
      finalChassisSpeeds.vxMetersPerSecond, finalChassisSpeeds.vyMetersPerSecond});

    SmartDashboard.putNumber("sidewaysInput", m_sidewaysSupplier.get());
    SmartDashboard.putNumber("forwardInput", m_forwardSupplier.get());

    SmartDashboard.putNumber("radius", radius);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private Rotation2d getTargetHeading(Vector2d robotLocation, Vector2d hubLocation) {
    double product = robotLocation.dot(hubLocation);
    double magnitudes = robotLocation.magnitude() * hubLocation.magnitude();
    double angle_rad = Math.acos(product / magnitudes);

    return new Rotation2d(angle_rad);
  }

  private double findStrafeX(double radius, double targetAngle, double max_velocity, double joystickInput, double constant) {
    return constant * max_velocity * joystickInput * -Math.sin(targetAngle);
  }

  private double findStrafeY(double radius, double targetAngle, double max_velocity, double joystickInput, double constant) {
    return constant * max_velocity * joystickInput * Math.cos(targetAngle);
  }

  private ChassisSpeeds averageChassisSpeeds(ChassisSpeeds arcChassisSpeeds, ChassisSpeeds forwardChassisSpeeds){
    SwerveModuleState[] arcStates = m_kinematics.toSwerveModuleStates(arcChassisSpeeds);
    SwerveModuleState[] forwardStates = m_kinematics.toSwerveModuleStates(forwardChassisSpeeds);
    SwerveModuleState[] finalModuleStates = new SwerveModuleState[4]; 

    for(int i=0; i<4; i++){
      SwerveModuleState arcState = arcStates[i];
      SwerveModuleState forwardState = forwardStates[i];
      SwerveModuleState finalState = finalModuleStates[i];

      Rotation2d arcStateRotation = arcState.angle;
      Rotation2d forwardStateRotation = forwardState.angle;

      //average of angles;
      Rotation2d finalStateRotation = new Rotation2d(
        (arcStateRotation.getRadians() + forwardStateRotation.getRadians()) /2);

      finalState.angle = finalStateRotation;
      finalState.speedMetersPerSecond = (arcState.speedMetersPerSecond + forwardState.speedMetersPerSecond)/2;
    }

    return m_kinematics.toChassisSpeeds(finalModuleStates);
  }
}