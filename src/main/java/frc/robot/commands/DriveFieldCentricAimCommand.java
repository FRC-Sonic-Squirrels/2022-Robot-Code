package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
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
    private final DoubleSupplier rotationSupplier;

    private ProfiledPIDController rotationalController = new ProfiledPIDController(3.0, 0.0, 0.02,
    new TrapezoidProfile.Constraints(
        Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        Drivetrain.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED * 0.9));

    public DriveFieldCentricAimCommand(Drivetrain drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier,
                               LimelightSubsystem limelight) {
        this.drivetrain = drivetrainSubsystem;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        this.limelight = limelight;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        //DO NOT MULTIPLY THESE BY MAX VELOCITY that is already done in robot container
        double rotationOutput = rotationSupplier.getAsDouble() * Constants.DriveFieldCentricConstant.ROTATION_MULTIPLIER;
        SmartDashboard.putNumber("Drive field centric rotation output", rotationOutput);

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

        // Multiply by max velocity to hopefully speed up the rotation of the robot 
        rotationOutput = rotationalController.calculate(drivetrain.getRotation().getRadians(), targetHeadingRadians) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

        if(Math.abs(rotationOutput) <0.05) { rotationOutput = 0.0; }

        drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationXSupplier.getAsDouble() * Constants.DriveFieldCentricConstant.TRANSLATION_MULTIPLIER,
                        translationYSupplier.getAsDouble() * Constants.DriveFieldCentricConstant.TRANSLATION_MULTIPLIER,
                        rotationOutput,
                        drivetrain.getRotation()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
