// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final LimeLight m_limeLight = new LimeLight();
    private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_limeLight);
    private final Intake m_intake = new Intake();

    private boolean isFieldRelative = true;
    private boolean isBalancing = false;
    private boolean isTrackingObject = false;
    private boolean isAvoidingObject = false;
    private boolean isIntakeOpen = true;
    double maxVel = Constants.PPvar.maxVel;
    double maxAccel = Constants.PPvar.maxAccel;
    double squareSize = Constants.PPvar.squareSize;
    double figureEightSize = Constants.PPvar.figureEightSize;
    double starSize = Constants.PPvar.starSize;
    private boolean isTrackingGoal = false;


    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    private void toggleFieldRelative() {
        isFieldRelative = !isFieldRelative;
    }

    private void toggleTracking() {
        if (!isAvoidingObject) {
            isTrackingObject = !isTrackingObject;
        }
    }

    private void toggleObjectAvoidance() {
        isAvoidingObject = !isAvoidingObject;
        isTrackingObject = true;
    }

    private void toggleBalancing() {
        isBalancing = !isBalancing;
        if (isBalancing) {
            isAvoidingObject = false;
            isTrackingObject = false;
            isFieldRelative = true;
        }
    }

    private void toggleIntake() {
        isIntakeOpen = !isIntakeOpen;
        if (isIntakeOpen) {
            m_intake.openIntake();
        } else {
            m_intake.closeIntake();
        }
    }

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        Command driveCommand = new RunCommand(() -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY() / 2,
                        OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(
                        m_driverController.getLeftX() / 2,
                        OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(
                        m_driverController.getRightX() / 2,
                        OIConstants.kDriveDeadband),
                isFieldRelative, true, isTrackingObject,
                isAvoidingObject, isBalancing),
                m_robotDrive);

        Command intakeCommand = new RunCommand(() -> m_intake.triggerInputHandler(
                MathUtil.applyDeadband(
                        m_driverController.getLeftTriggerAxis() / 4,
                        OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(
                        m_driverController.getRightTriggerAxis() / 4,
                        OIConstants.kDriveDeadband)), m_intake);

        // Configure default commands
        m_robotDrive.setDefaultCommand(
                new ParallelCommandGroup(driveCommand, intakeCommand)
        );
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        new JoystickButton(m_driverController, Button.kRightBumper.value)
                .toggleOnTrue(new InstantCommand(
                        this::toggleIntake));

        new JoystickButton(m_driverController, Button.kB.value)
                .toggleOnTrue(new InstantCommand(
                        this::toggleBalancing));

        new JoystickButton(m_driverController, Button.kA.value)
                .toggleOnTrue(new InstantCommand(
                        m_robotDrive::zeroHeading,
                        m_robotDrive));

        new JoystickButton(m_driverController, Button.kY.value)
                .toggleOnTrue(new InstantCommand(
                        this::toggleFieldRelative));

        new JoystickButton(m_driverController, Button.kX.value)
                .toggleOnTrue(new InstantCommand(
                        this::toggleTracking));

        /*
         * new JoystickButton(m_driverController, Button.kRightBumper.value)
         * .whileTrue(new RunCommand(
         * () -> m_robotDrive.setCounterMovement(1),
         * m_robotDrive));
         *
         * new JoystickButton(m_driverController, Button.kLeftBumper.value)
         * .whileTrue(new RunCommand(
         * () -> m_robotDrive.setCounterMovement(0),
         * m_robotDrive));
         */
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        /**
         //one meter forward
         PathPlannerTrajectory oneMeter = PathPlanner.loadPath("one meter", new PathConstraints(4, 3));
         PathPlannerState oneMeterState = (PathPlannerState) oneMeter.sample(1.2);
         System.out.println(oneMeterState.velocityMetersPerSecond);
         return m_robotDrive.followTrajectoryCommand(oneMeter, true);
         **/

        //90 deg
        PathPlannerTrajectory ninetyDeg = PathPlanner.loadPath("New Path Copy", new PathConstraints(maxVel, maxAccel));

        // Move forward 2 feet, turn 180 degrees, move forward 2 feet, reverse 2 feet
        PathPlannerTrajectory task1 = PathPlanner.generatePath(
                new PathConstraints(0.5, 0.2),
                new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0),
                        Rotation2d.fromDegrees(0)),
                new PathPoint(new Translation2d(0.61, 0), Rotation2d.fromDegrees(0),
                        Rotation2d.fromDegrees(0)),
                new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0),
                        Rotation2d.fromDegrees(0)),
                new PathPoint(new Translation2d(0.61, 0), Rotation2d.fromDegrees(0),
                        Rotation2d.fromDegrees(0)));
        // return m_robotDrive.followTrajectoryCommand(task1, true);

        //square
        PathPlannerTrajectory square = PathPlanner.generatePath(
                new PathConstraints(maxVel, maxAccel),
                new PathPoint(new Translation2d(0, 0),
                        Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
                new PathPoint(new Translation2d(0, squareSize),
                        Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
                new PathPoint(new Translation2d(-squareSize, squareSize),
                        Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
                new PathPoint(new Translation2d(-squareSize, 0),
                        Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
                new PathPoint(new Translation2d(0, 0),
                        Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))
        );
        //return m_robotDrive.followTrajectoryCommand(square, true);

        //figure 8
        PathPlannerTrajectory figureEight = PathPlanner.generatePath(
                new PathConstraints(maxVel, maxAccel),
                new PathPoint(new Translation2d(0, 0),
                        Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(45)),
                new PathPoint(new Translation2d(figureEightSize / 2, 0),
                        Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(-90)),
                new PathPoint(new Translation2d(0, 0),
                        Rotation2d.fromDegrees(-45), Rotation2d.fromDegrees(-45)),
                new PathPoint(new Translation2d(-figureEightSize / 2, 0),
                        Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(-90)),
                new PathPoint(new Translation2d(0, 0),
                        Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(45))
        );
        //return m_robotDrive.followTrajectoryCommand(figureEight, true);

        //star
        /**PathPlannerTrajectory star = PathPlanner.generatePath(
         new PathConstraints(4, 3),
         new PathPoint(new Translation2d(Constants.shapeSizes.figureEightSize*Math.sin(72),
         Constants.shapeSizes.figureEightSize*Math.sin(72)),
         Rotation2d.fromDegrees(72),
         Rotation2d.fromDegrees(72)),
         new PathPoint(new Translation2d(Constants.shapeSizes.figureEightSize*Math.sin(72),
         );**/

        //PID tune
        PathPlannerTrajectory pidTune = PathPlanner.generatePath(
                new PathConstraints(maxVel, maxAccel),
                new PathPoint(new Translation2d(0, 0),
                        Rotation2d.fromDegrees(0),
                        Rotation2d.fromDegrees(0)),

                new PathPoint(new Translation2d(Units.feetToMeters(2), 0),
                        Rotation2d.fromDegrees(0),
                        Rotation2d.fromDegrees(0))
        );

        //turn tune
        PathPlannerTrajectory pidTurn = PathPlanner.generatePath(
                new PathConstraints(maxVel, maxAccel),
                new PathPoint(new Translation2d(0, 0),
                        Rotation2d.fromDegrees(0),
                        Rotation2d.fromDegrees(0)),

                new PathPoint(new Translation2d(0, 0),
                        Rotation2d.fromDegrees(0),
                        Rotation2d.fromDegrees(90))
        );

        return m_robotDrive.followTrajectoryCommand(pidTurn, true);
    }

    public Command getLeftPlaceMoveCommand() {
        PathPlannerTrajectory LeftPlaceMove = PathPlanner.generatePath(
                new PathConstraints(maxVel, maxAccel),
                new PathPoint(new Translation2d(0, 0),
                        Rotation2d.fromDegrees(0),
                        Rotation2d.fromDegrees(0)),

                new PathPoint(new Translation2d(-0.6096, 0),
                        Rotation2d.fromDegrees(0),
                        Rotation2d.fromDegrees(0))
        );
        return m_robotDrive.followTrajectoryCommand(LeftPlaceMove, true);
    }

    public Command getRightPlaceMoveCommand() {
        PathPlannerTrajectory RightPlaceMove = PathPlanner.generatePath(
                new PathConstraints(maxVel, maxAccel),
                new PathPoint(new Translation2d(0, 0),
                        Rotation2d.fromDegrees(0),
                        Rotation2d.fromDegrees(0)),

                new PathPoint(new Translation2d(0.6096, 0),
                        Rotation2d.fromDegrees(0),
                        Rotation2d.fromDegrees(0))
        );
        return m_robotDrive.followTrajectoryCommand(RightPlaceMove, true);
    }

    /*public Command getAutoAlignAndPlaceCommand() {
        Command autoAlign = new RunCommand(m_robotDrive::calculateTurnTo180AngularVelocity, m_robotDrive).until(() -> m_robotDrive.calculateTurnTo180AngularVelocity() == 0);
    }*/
}