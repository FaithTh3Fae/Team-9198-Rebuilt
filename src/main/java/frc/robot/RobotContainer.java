package frc.robot;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.commands.SwerveJoystickCmd;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final Shooter shooter = new Shooter();
    private final Arm arm = new Arm();
    private final Climber climber = new Climber();

    private final static Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    private final static Joystick xbox = new Joystick(ScoringConstants.kScoringControllerPort);

    private final SendableChooser<Integer> chooser = new SendableChooser<>();

    public RobotContainer() {

    chooser.setDefaultOption("Left Coral", 0);
    chooser.addOption("Center Coral", 1);
    chooser.addOption("Right Coral", 2);
    chooser.addOption("Right Back Coral", 3);
    chooser.addOption("Left Back Coral", 4);
    chooser.addOption("Taxi", 5);
    chooser.addOption("Nothing", 6);
    
    SmartDashboard.putString("Auto Mode:", "Filler Text");
    SmartDashboard.putData("Auto Mode", chooser);

    configureButtonBindings();

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
        () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
        () -> -driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
        () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
        () -> driverJoytick.getRawAxis(OIConstants.kDriverThrottleAxis),
        () -> driverJoytick.getRawButton(OIConstants.kDriverSlowTurnButtonIdx)
    ));
}

final boolean getisShoot(int autoMode) {
    boolean isShoot;
    switch (autoMode) {
        case 0, 1, 2, 3, 4:
            isShoot = true;
            break;
        default:
            isShoot = false;
            break;
    }
    return isShoot;
}

final List<Pose2d> getWayPoints(Pose2d currentPose, int autoMode) {
    List<Pose2d> awp = new ArrayList<>();
    ArrayList<ArrayList<Double>> wpXY = new ArrayList<>();

    switch (autoMode) {
        case 0, 2:
            wpXY.add(new ArrayList<>(Arrays.asList(6.0,0.0)));
            break;
        case 1, 5:
            wpXY.add(new ArrayList<>(Arrays.asList(2.5,0.0)));
            break;
        case 3:
            wpXY.add(new ArrayList<>(Arrays.asList(-6.0,0.0)));
            wpXY.add(new ArrayList<>(Arrays.asList(-6.0,3.0)));
            wpXY.add(new ArrayList<>(Arrays.asList(-4.0,3.0)));
            break;
        case 4:
            wpXY.add(new ArrayList<>(Arrays.asList(-6.0,0.0)));
            wpXY.add(new ArrayList<>(Arrays.asList(-6.0,-3.0)));
            wpXY.add(new ArrayList<>(Arrays.asList(-4.0,-3.0)));
            break;
        default:
            wpXY.add(new ArrayList<>(Arrays.asList(0.0,0.0)));
            break;
    }
    awp.clear();
    awp.add(new Pose2d(currentPose.getTranslation(), new Rotation2d()));
    for (int i = 0; i < wpXY.size(); i++) {
        awp.add(new Pose2d(currentPose.getTranslation().plus(new Translation2d(wpXY.get(i).get(0),wpXY.get(i).get(1))), new Rotation2d()));
    }
    return awp;
}

final Command ShootPiece = new ParallelCommandGroup(
    shooter.PrepareShooter(0.4)
);

final Command AlageIntake = new ParallelCommandGroup(
    shooter.PrepareShooter(-0.4)
);

final Command ArmDown = new ParallelCommandGroup(
    arm.PrepareArm(-0.2)
);

final Command ArmUp = new ParallelCommandGroup(
    arm.PrepareArm(0.2)
);

final Command ArmIdle = new ParallelCommandGroup(
    arm.PrepareArm(0.06)
);

final Command ClimbPrepare = new SequentialCommandGroup(
    new InstantCommand(() -> climber.runClimbMotor(-0.75)).raceWith(new InstantCommand(() -> arm.runArmMotor(0.075))),
    new WaitCommand(2.25),
    new InstantCommand(() -> climber.climbStop())
);

final Command ClimbEndgame = new SequentialCommandGroup(
    new InstantCommand(() -> climber.runClimbMotor(0.4)).raceWith(new InstantCommand(() -> arm.runArmMotor(0.075))),
    new WaitCommand(10),
    new InstantCommand(() -> climber.climbStop())
);

// Temporary commands to test climb motor
final Command tempClimbF = new SequentialCommandGroup(
    climber.PrepareClimber(-0.4)
);
final Command tempClimbB = new ParallelCommandGroup(
    climber.PrepareClimber(0.4)
);

    private void configureButtonBindings() {
        new JoystickButton(driverJoytick, OIConstants.kDriverResetGyroButtonIdx).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
        final JoystickButton outtakeButton = new JoystickButton(xbox, 1);
        outtakeButton.whileTrue(ShootPiece);
        final JoystickButton intakeButton = new JoystickButton(xbox, 4);
        intakeButton.whileTrue(AlageIntake);
        final JoystickButton armDownButton = new JoystickButton(xbox, 5);
        armDownButton.whileTrue(ArmDown);
        armDownButton.whileFalse(ArmIdle);
        final JoystickButton armUpButton = new JoystickButton(xbox, 6);
        armUpButton.whileTrue(ArmUp);
        final JoystickButton climbPrepareButton = new JoystickButton(xbox, 7);
        climbPrepareButton.onTrue(ClimbPrepare);
        final JoystickButton climbEndgameButton = new JoystickButton(xbox, 8);
        climbEndgameButton.onTrue(ClimbEndgame);

        // Temporary
        final JoystickButton tempP = new JoystickButton(xbox, 3);
        tempP.whileTrue(tempClimbB);
        final JoystickButton tempE = new JoystickButton(xbox, 2);
        tempE.whileTrue(tempClimbF);
    }

     public Command getAutonomousCommand() {

        // // 1. Create trajectory settings
        // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        //         AutoConstants.kMaxSpeedMetersPerSecond,
        //         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //                 .setKinematics(DriveConstants.kDriveKinematics);

        // // 2. Generate trajectory
        // // Note: -y value is to the left (field relative)
        // // This sa,ple is an example of a figure 8 auto path
        // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(0, 0, new Rotation2d(0)),
        //         List.of(
        //                 new Translation2d(.7, -0.7),
        //                 new Translation2d(1.2, 0),
        //                 new Translation2d(.7, 0.7),
        //                 new Translation2d(0, 0),
        //                 new Translation2d(-.7, -0.7),
        //                 new Translation2d(-1.2, 0),
        //                 new Translation2d(-.7, 0.7)
        //                 ),
        //         new Pose2d(
        //        0, 0
        //         , Rotation2d.fromDegrees(0)),
        //         trajectoryConfig);      


        // // 3. Define PID controllers for tracking trajectory
        // PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        // PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        // ProfiledPIDController thetaController = new ProfiledPIDController(
        //         AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // // 4. Construct command to follow trajectory
        // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        //         trajectory,
        //         swerveSubsystem::getPose,
        //         DriveConstants.kDriveKinematics,
        //         xController,
        //         yController,
        //         thetaController,
        //         swerveSubsystem::setModuleStates,
        //         swerveSubsystem);

        // // 5. Add some init and wrap-up, and return everything
        // return new SequentialCommandGroup(
        //         new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
        //         swerveControllerCommand,
        //         new InstantCommand(() -> swerveSubsystem.stopModules()));

        // return chooser.getSelected();
        List<Pose2d> autoWaypoints;
        Pose2d currentPose = swerveSubsystem.getPose();
        int autoMode = chooser.getSelected();

        autoWaypoints = getWayPoints(currentPose, autoMode);
        boolean isShoot = getisShoot(autoMode);

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(autoWaypoints);

        if (waypoints.isEmpty()) {
            throw new IllegalStateException("No waypoints generated");
        }

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            new PathConstraints(
                1.5,
                1.5,
                Units.degreesToRadians(360),
                Units.degreesToRadians(540)
            ),
            null,
            new GoalEndState(0.0, currentPose.getRotation())
            );
        path.preventFlipping = true;
        if (isShoot) {
            return new SequentialCommandGroup(
                AutoBuilder.followPath(path),
                new InstantCommand(() -> shooter.runShootMotor(0.4)),
                new WaitCommand(0.35),
                new InstantCommand(() -> arm.runArmMotor(-0.25)),
                new WaitCommand(0.4),
                new InstantCommand(() -> shooter.runShootMotor(0)).raceWith(new InstantCommand(() -> arm.runArmMotor(0))),
                new WaitCommand(1.0),
                new InstantCommand(() -> arm.runArmMotor(0.25)),
                new WaitCommand(0.5),
                new InstantCommand(() -> arm.runArmMotor(0))
            );
        } else {
            return AutoBuilder.followPath(path);
        }
}

    public static Joystick getDriverJoytick() {
        return driverJoytick;
    }
}
