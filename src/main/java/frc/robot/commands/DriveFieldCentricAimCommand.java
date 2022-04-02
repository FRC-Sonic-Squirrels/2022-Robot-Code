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
    private final DoubleSupplier rotationSupplier;

    //private LinearFilter rotationFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    // private ProfiledPIDController rotationalController = new ProfiledPIDController(3.0, 0.0, 0.02,
    // new TrapezoidProfile.Constraints(
    //     Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
    //     Drivetrain.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED * 0.9));

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

        double rotationOutput = rotationSupplier.getAsDouble() * Constants.DriveFieldCentricConstant.ROTATION_MULTIPLIER;
        SmartDashboard.putNumber("LLRS driver rotation output", rotationOutput);

        if (limelight.seesTarget()) {
            rotationOutput = rotationOutput * 0.1 - (limelight.targetYaw() / 27.0 ) * 0.6;
        } 

        // Just for debugging 
        SmartDashboard.putNumber("LLRS heading ll", Math.toDegrees(limelight.getTargetHeadingRadians()));
        SmartDashboard.putNumber("LLRS heading odometry",
                        SwerveUtils.headingToPoint(drivetrain.getPose(),
                                        Constants.FieldConstants.HUB_CENTER, new Rotation2d(0))
                                .getDegrees());

        // deadband the rotation to avoid oscillation
        if (Math.abs(rotationOutput) < 0.05) {
            rotationOutput = 0.0;
        }

        SmartDashboard.putNumber("LLRS rotation output", rotationOutput);

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
