// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.team2930.lib.util.SwerveTestTrajectories;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoEngage;
import frc.robot.commands.DriveWithSetRotationCommand;
import frc.robot.commands.IntakeDeployCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.commands.ShootWithSetRPM;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SwerveTrajectoryAutonomousCommandFactory {

    private static ShooterSubsystem m_shooter;
    private static Drivetrain m_drivetrain;
    private static CargoSubsystem m_cargo;
    private static IntakeSubsystem m_intake;
    private static SwerveTestTrajectories m_tt;
    private static Robot m_robot;


    public SwerveTrajectoryAutonomousCommandFactory(Drivetrain drivetrain, ShooterSubsystem shooter,
            CargoSubsystem cargo, IntakeSubsystem intake, Robot robot, double maxVelocity,
            double maxAcceleration) {

        m_drivetrain = drivetrain;
        m_shooter = shooter;
        m_cargo = cargo;
        m_intake = intake;
        m_robot = robot;

        m_tt = new SwerveTestTrajectories(maxVelocity, maxAcceleration,
                Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, drivetrain.kinematics());
    }

    public Command curve() {

        PathPlannerTrajectory path = PathPlanner.loadPath("test_curve", AutoConstants.maxVelocity,
                AutoConstants.maxAcceleration);

        return PPSwerveControlCommand(path, true).beforeStarting(
                new InstantCommand(() -> m_drivetrain.resetOdometry(path.getInitialPose())));
    }

    public Command straightLine() {

        PathPlannerTrajectory path = PathPlanner.loadPath("test_straightline", 1.5, 0.75);

        return PPSwerveControlCommand(path, true).beforeStarting(
                new InstantCommand(() -> m_drivetrain.resetOdometry(path.getInitialPose())));
    }


    /**
     * Create a swerve trajectory follow command. If stopAtEnd is set to true, robot will come to
     * full stop at the end of the command.
     * 
     * @param trajectory
     * @param stopAtEnd
     * @return trajectoryFollowCommand
     */
    public static Command SwerveControllerCommand(Trajectory trajectory, boolean stopAtEnd) {

        // Example from WPILib:
        // https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervecontrollercommand/RobotContainer.java

        var thetaController = new ProfiledPIDController(AutoConstants.kPThetaController,
                AutoConstants.kIThetaController, AutoConstants.kDThetaController,
                AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Command swerveControllerCommand = new SwerveControllerCommand(trajectory,
                m_drivetrain::getPose, m_drivetrain.kinematics(),
                new PIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD),
                new PIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD),
                thetaController, m_drivetrain::setModuleStates, m_drivetrain);

        if (stopAtEnd) {
            // Stop at the end. A good safe default, but not desireable if running two paths back to
            // back
            swerveControllerCommand = swerveControllerCommand
                    .andThen(() -> m_drivetrain.drive(new ChassisSpeeds(0, 0, 0)));
        }


        return swerveControllerCommand;
    }



    // ------------ 2023 charged up autos ------------

//     public Command driveAutoEngage(boolean flip) {
//         return new SequentialCommandGroup(
//                 Commands.print("----------------------------DRIVE AUTO ENGAGE ALLIANCE: "
//                         + DriverStation.getAlliance() + "-----------"),
//                 // !!!!!!NEGATIVE NUMBER FOR X VELOCITY BECAUSE JOYSTICK VALUE
//                 new ConditionalCommand(
//                         new DriveWithSetRotationCommand(m_drivetrain, () -> (1.2), () -> 0,
//                                 () -> -1, Math.toRadians(180)).until(
//                                         () -> Math.abs(m_drivetrain.getGyroscopePitch()) >= 13.5)
//                                         .withTimeout(0.95),
//                         new DriveWithSetRotationCommand(m_drivetrain, () -> (-1.2), () -> 0,
//                                 () -> -1, Math.toRadians(180)).until(
//                                         () -> Math.abs(m_drivetrain.getGyroscopePitch()) >= 13.5)
//                                         .withTimeout(0.95),
//                         () -> flip),
//                 new ConditionalCommand(
//                         new DriveWithSetRotationCommand(m_drivetrain, () -> (1.5), () -> 0,
//                                 () -> -1, Math.toRadians(180)).withTimeout(0.25), // 0.175
//                         new DriveWithSetRotationCommand(m_drivetrain, () -> (-1.5), () -> 0,
//                                 () -> -1, Math.toRadians(180)).withTimeout(0.25), // 0.175,
//                         () -> flip),
//                 new ConditionalCommand(new AutoEngage(m_drivetrain, true),
//                         new AutoEngage(m_drivetrain, false),
//                         () -> DriverStation.getAlliance() == Alliance.Red)
//                                 .handleInterrupt(() -> m_drivetrain.setXStance()));
//     }

    /**
     * eventMap() - generate a fresh PathPlanner EventMap
     *
     * @return EventMap
     */
    private HashMap<String, Command> getEventMap() {
        HashMap<String, Command> eventMap = new HashMap<>();

        eventMap.put("scoreLow", new SequentialCommandGroup(new InstantCommand(()->{
                m_intake.setStopMode();
                m_cargo.setReverseMode();
        }).withTimeout(2),
        new InstantCommand(()->{
                m_cargo.setStopMode();}
        )));
        eventMap.put("scoreHigh", new ShootWithSetRPM(Constants.ShooterConstants.HIGH_NODE_RPM, m_cargo, m_shooter, m_robot));
        eventMap.put("scoreMid", new ShootWithSetRPM(Constants.ShooterConstants.MID_NODE_RPM, m_cargo, m_shooter, m_robot));

        eventMap.put("deployIntake", new InstantCommand(() -> {
            m_intake.deployIntake();
            m_intake.setForwardMode();
            m_cargo.setIntakeMode();
        }));
        eventMap.put("retractIntake", new InstantCommand(() -> {
            m_intake.retractIntake();
            m_intake.setStopMode();
            m_cargo.setIdleMode();
        }));

        return eventMap;
    }

    public Command scoreHigh() {
        return new ShootWithSetRPM(Constants.ShooterConstants.HIGH_NODE_RPM, m_cargo, m_shooter, m_robot).withTimeout(5);
    }

    public Command scoreLow() {
        return new IntakeReverseCommand(m_intake, m_cargo).withTimeout(1);
    }

    public Command hp2piece() {
        PathPlannerTrajectory hp2piece = PathPlanner.loadPath("hp2piece",
                Constants.AutoConstants.maxVelocity, Constants.AutoConstants.maxAcceleration);

        return new SequentialCommandGroup(
                new InstantCommand(
                        () -> m_drivetrain.resetOdometry(
                                getStartPoseFor2023Paths(hp2piece, DriverStation.getAlliance())),
                        m_drivetrain),

                getEventMap().get("scoreHigh"),

                new FollowPathWithEvents(PPSwerveControlCommand(hp2piece, true, true),
                        hp2piece.getMarkers(), getEventMap()),

                getEventMap().get("scoreMid")

        );
    }

//     public Command hp2pieceEngage() {
//         PathPlannerTrajectory hp2pieceEngage = PathPlanner.loadPath("hp2pieceEngage",
//                 Constants.AutoConstants.maxVelocity, Constants.AutoConstants.maxAcceleration);

//         return new SequentialCommandGroup(hp2piece(),
//                 PPSwerveControlCommand(hp2pieceEngage, true, true), driveAutoEngage(false)

//         );
//     }

//     public Command hp3piece() {
//         PathPlannerTrajectory hp3piece = PathPlanner.loadPath("hp3piece",
//                 Constants.AutoConstants.maxVelocity, Constants.AutoConstants.maxAcceleration);

//         return new SequentialCommandGroup(hp2piece(),
//         new FollowPathWithEvents(PPSwerveControlCommand(hp3piece, true, true),
//         hp3piece.getMarkers(), getEventMap())
//         );
//     }

    public Command wall2piece() {
        PathPlannerTrajectory wall2piece = PathPlanner.loadPath("wall2piece",
                Constants.AutoConstants.maxVelocity, Constants.AutoConstants.maxAcceleration);
        return new SequentialCommandGroup(
                new InstantCommand(
                        () -> m_drivetrain.resetOdometry(
                                getStartPoseFor2023Paths(wall2piece, DriverStation.getAlliance())),
                        m_drivetrain),

                getEventMap().get("scoreHigh"),

                new FollowPathWithEvents(PPSwerveControlCommand(wall2piece, true, true),
                        wall2piece.getMarkers(), getEventMap()),

                getEventMap().get("scoreMid"));
    }

//     public Command wall2pieceEngage() {
//         PathPlannerTrajectory wall2pieceEngage = PathPlanner.loadPath("wall2pieceEngage",
//                 Constants.AutoConstants.maxVelocity, Constants.AutoConstants.maxAcceleration);

//         return new SequentialCommandGroup(hp2piece(),
//                 PPSwerveControlCommand(wall2pieceEngage, true, true), driveAutoEngage(false));
//     }

//     public Command wall3piece() {
//         PathPlannerTrajectory wall3piece = PathPlanner.loadPath("wall3piece",
//                 Constants.AutoConstants.maxVelocity, Constants.AutoConstants.maxAcceleration);

//         return new SequentialCommandGroup(wall2piece(),
//         new FollowPathWithEvents(PPSwerveControlCommand(wall3piece, true, true),
//         wall3piece.getMarkers(), getEventMap())
//         );
//     }

    public Command middle1pieceEngage() {
        return new SequentialCommandGroup(

                        new InstantCommand(() -> m_drivetrain.resetOdometry(new Pose2d(0,0, Rotation2d.fromDegrees(180))), m_drivetrain),


                        new IntakeReverseCommand(m_intake, m_cargo).withTimeout(1),
                        // getEventMap().get("scoreHigh"),

                        // Commands.runEnd(
                            new DriveWithSetRotationCommand(m_drivetrain, () -> 2.0, () ->0.0, ()-> -1, Math.PI)
                        //     () -> m_drivetrain.stop(),
                        //     m_drivetrain)
                        .until(() -> Math.abs(m_drivetrain.getGyroscopePitch()) > 15)
                        .withTimeout(2),
                        // Commands.runEnd(
                        //     () -> m_drivetrain.drive(2.0, 0.0, 0.0),
                        //     () -> m_drivetrain.stop(),
                        //     m_drivetrain)
                        new DriveWithSetRotationCommand(m_drivetrain, () -> 1.5, () ->0.0, ()-> -1, Math.PI)
                        .until(
                            new Trigger(() -> Math.abs(m_drivetrain.getGyroscopePitch()) < 3).debounce(0.45))
                        .withTimeout(2.2),

                        new DriveWithSetRotationCommand(m_drivetrain, () -> -2.0, () ->0.0, ()-> -1, Math.PI)
                        .until(() -> Math.abs(m_drivetrain.getGyroscopePitch()) > 15)
                        .withTimeout(2.0),
                    new AutoEngage(m_drivetrain, false)
                        .handleInterrupt(() -> m_drivetrain.setXStance())
                        );
    }

    public Command rightTaxi(){
        PathPlannerTrajectory rightTaxi = PathPlanner.loadPath("rightTaxi",
                Constants.AutoConstants.maxVelocity, Constants.AutoConstants.maxAcceleration);
        return new SequentialCommandGroup(
                getEventMap().get("scoreLow"),
                new FollowPathWithEvents(PPSwerveControlCommand(rightTaxi, true, false), rightTaxi.getMarkers(), getEventMap())
        );
    }

    public Command leftTaxi(){
        PathPlannerTrajectory leftTaxi = PathPlanner.loadPath("leftTaxi",
                Constants.AutoConstants.maxVelocity, Constants.AutoConstants.maxAcceleration);
        return new SequentialCommandGroup(
                getEventMap().get("scoreLow"),
                new FollowPathWithEvents(PPSwerveControlCommand(leftTaxi, true, false), leftTaxi.getMarkers(), getEventMap())
        );
    }

    public Command sideTaxi(){
        return new SequentialCommandGroup(
                new InstantCommand(() -> m_drivetrain.resetOdometry(new Pose2d(0,0, Rotation2d.fromDegrees(180))), m_drivetrain),
                new IntakeReverseCommand(m_intake, m_cargo).withTimeout(1),
                new DriveWithSetRotationCommand(m_drivetrain, () -> 1.0, () ->0.0, ()-> -1, Math.PI).withTimeout(3.4)
        );
    }

    public static Command PPSwerveControlCommand(PathPlannerTrajectory traj, boolean stopAtEnd,
            boolean useAllianceColor) {
        var thetaController = new PIDController(AutoConstants.kPThetaController,
                AutoConstants.kIThetaController, AutoConstants.kDThetaController);

        Command swerveControllerCommand = new PPSwerveControllerCommand(traj, m_drivetrain::getPose,
                m_drivetrain.kinematics(),
                new PIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD),
                new PIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD),
                thetaController, m_drivetrain::setModuleStates, useAllianceColor, m_drivetrain);

        if (stopAtEnd) {
            // Stop at the end. A good safe default, but not desireable if running two paths back to
            // back
            swerveControllerCommand = swerveControllerCommand
                    .andThen(() -> m_drivetrain.drive(new ChassisSpeeds(0, 0, 0)));
        }
        return swerveControllerCommand;
    }

    /**
     * Create a swerve trajectory follow command. If stopAtEnd is set to true, robot will come to
     * full stop when done.
     * 
     * @param trajectory
     * @param stopAtEnd
     * @return
     */
    public static Command PPSwerveControlCommand(PathPlannerTrajectory trajectory,
            boolean stopAtEnd) {

        // var thetaController =
        // new ProfiledPIDController(AutoConstants.kPThetaController,
        // AutoConstants.kIThetaController,
        // AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Command swerveControllerCommand =
        // new PPSwerveControllerCommand(trajectory, m_drivetrain::getPose,
        // m_drivetrain.kinematics(),
        // new PIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD),
        // new PIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD),
        // thetaController, m_drivetrain::setModuleStates, m_drivetrain);

        // 2023 path planner doesnt take profiled PIDController for thetaController
        var thetaController = new PIDController(AutoConstants.kPThetaController,
                AutoConstants.kIThetaController, AutoConstants.kDThetaController);

        Command swerveControllerCommand = new PPSwerveControllerCommand(trajectory,
                m_drivetrain::getPose, m_drivetrain.kinematics(),
                new PIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD),
                new PIDController(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD),
                thetaController, m_drivetrain::setModuleStates, m_drivetrain);

        if (stopAtEnd) {
            // Stop at the end. A good safe default, but not desireable if running two paths back to
            // back
            swerveControllerCommand = swerveControllerCommand
                    .andThen(() -> m_drivetrain.drive(new ChassisSpeeds(0, 0, 0)));
        }
        return swerveControllerCommand;
    }

    /**
     * This returns the pose2d to reset the odometry to at the start of auto. If you just use
     * path.getInitalPose() the rotation is the angle of the heading not what the robot is facing.
     * For swerve robots the rotation value at a state is the
     * path.getInitialState().holonomicRotation
     * 
     * @param path to get the starting pose of
     * @return the pose2d to reset the odometry to
     */
    public static Pose2d getStartPoseForPath(PathPlannerTrajectory path) {
        return new Pose2d(path.getInitialPose().getTranslation(),
                path.getInitialState().holonomicRotation);
    }

    public static Pose2d getStartPoseFor2023Paths(PathPlannerTrajectory path, Alliance alliance) {
        var transformedPath = PathPlannerTrajectory.transformTrajectoryForAlliance(path, alliance);

        return getStartPoseForPath(transformedPath);
    }

}
