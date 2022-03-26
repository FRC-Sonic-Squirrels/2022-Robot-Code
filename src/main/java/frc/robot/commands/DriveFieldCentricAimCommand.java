package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import java.util.function.DoubleSupplier;
import com.team2930.lib.util.SwerveUtils;

public class DriveFieldCentricAimCommand extends CommandBase {
    
    private final Drivetrain drivetrain;
    private final LimelightSubsystem limelight;

    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;

    private LinearFilter rotationFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    private ProfiledPIDController rotationalController = new ProfiledPIDController(3.0, 0.0, 0.02,
    new TrapezoidProfile.Constraints(
        Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        Drivetrain.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED * 0.9));

    public DriveFieldCentricAimCommand(Drivetrain drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               LimelightSubsystem limelight) {
        this.drivetrain = drivetrainSubsystem;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.limelight = limelight;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        double targetHeadingRadians;

        if (limelight.seesTarget()) {
            // get heading from limelight
            targetHeadingRadians = limelight.getTargetHeadingRadians();
        } else {
            // get heading from odometry
            targetHeadingRadians =
                    SwerveUtils.headingToPoint(drivetrain.getPose(),
                                               Constants.FieldConstants.HUB_CENTER,
                                               new Rotation2d(Math.PI))
                                .getRadians();
        }

        double filteredTargetRotation = rotationFilter.calculate(targetHeadingRadians);

        // Multiply by max velocity to hopefully speed up the rotation of the robot 
        double rotationOutput = rotationalController.calculate(
            drivetrain.getRotation().getRadians(), filteredTargetRotation) * 
              Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

        // deadband the rotation to avoid oscillation
        if (Math.abs(rotationOutput) < 0.05) {
            rotationOutput = 0.0;
        }

        drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationXSupplier.getAsDouble() * Constants.DriveFieldCentricConstant.TRANSLATION_MULTIPLIER,
                        translationYSupplier.getAsDouble() * Constants.DriveFieldCentricConstant.TRANSLATION_MULTIPLIER,
                        rotationOutput,
                        drivetrain.getRotation()
                )
        );

        SmartDashboard.putNumber("Drive rotationalOutput", rotationOutput);
        SmartDashboard.putNumber("Drive targetHeading", Math.toRadians(targetHeadingRadians));
        SmartDashboard.putNumber("Drive targetHeading filtered", Math.toRadians(filteredTargetRotation));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
