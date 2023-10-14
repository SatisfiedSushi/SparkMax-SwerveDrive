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
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
//import com.pathplanner.lib.PathConstraints;
//import com.pathplanner.lib.PathPlanner;
//import com.pathplanner.lib.PathPlannerTrajectory;
//import com.pathplanner.lib.PathPoint;

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

    private void toggleCam() {
        if (m_limeLight.getPipline() == 7.0) {
            m_limeLight.setPipeline(8);
        } else {
            m_limeLight.setPipeline(7);
        }
    }

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        Command driveCommand = new RunCommand(() -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(),
                        OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(
                        m_driverController.getLeftX(),
                        OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(
                        m_driverController.getRightX(),
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

        new JoystickButton(m_driverController, Button.kLeftBumper.value)
                .toggleOnTrue(new InstantCommand(
                        this::toggleCam));

        //new POVButton(m_driverController, 90).onTrue(getAutoAlignAndPlaceCommand("right"));
        //new POVButton(m_driverController, 270).onTrue(getAutoAlignAndPlaceCommand("left"));
        //new POVButton(m_driverController, 0).onTrue(getAutoAlignAndPlaceCommand("middle"));

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

    

    /*public Command getLeftPlaceMoveCommand() {
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

    public Command moveDistanceY(double distance) {
        PathPlannerTrajectory moveDistanceY = PathPlanner.generatePath(
                new PathConstraints(maxVel, maxAccel),
                new PathPoint(new Translation2d(0, 0),
                        Rotation2d.fromDegrees(0),
                        Rotation2d.fromDegrees(0)),

                new PathPoint(new Translation2d(0, distance),
                        Rotation2d.fromDegrees(0),
                        Rotation2d.fromDegrees(0))
        );
        return m_robotDrive.followTrajectoryCommand(moveDistanceY, true);
    }*/

    //TODO: SWITCH TO LOCALIZATION IF HAVE TIME BECAUSE THIS AINT WORKING
    /*public Command getAutoAlignAndPlaceCommand(String location) {
        isFieldRelative = false;
        isTrackingObject = false;
        Command autoAlign = new RunCommand(() -> m_robotDrive.drive(0, 0, m_robotDrive.calculateTurnTo180AngularVelocity(), isFieldRelative, true, false,
                false, false), m_robotDrive).until(() -> m_robotDrive.calculateTurnTo180AngularVelocity() == 0);
        Command setPipelineToTags = new InstantCommand(() -> m_limeLight.setPipeline(Constants.kIsTeamBlue ? 2 : 3), m_limeLight);
        Command trackApriltag = new RunCommand(() -> m_robotDrive.drive(0, 0, 0, isFieldRelative, true, true,
                false, false), m_robotDrive, m_limeLight).until(() -> m_robotDrive.calculateTurnTo180AngularVelocity() == 0);
        Command moveIntoPlace = moveDistanceY(m_limeLight.calculateDistanceToTargetMeters(Constants.LimelightConstants.kLLHeight,
                Constants.LimelightConstants.kObjectHeight,
                Constants.LimelightConstants.kLLPitch,
                Math.toRadians(m_limeLight.getYAngle())) - 5).until(() -> m_limeLight.calculateDistanceToTargetMeters(Constants.LimelightConstants.kLLHeight,
                Constants.LimelightConstants.kObjectHeight,
                Constants.LimelightConstants.kLLPitch,
                Math.toRadians(m_limeLight.getYAngle())) <= 5);
        Command moveToPlace = moveDistanceY(0.3);

        switch (location){
            case "left":
                return new SequentialCommandGroup(autoAlign, setPipelineToTags, trackApriltag, moveIntoPlace, getLeftPlaceMoveCommand(), moveToPlace, new InstantCommand(m_intake::openIntake, m_intake));
            case "right":
                return new SequentialCommandGroup(autoAlign, setPipelineToTags, trackApriltag, moveIntoPlace, getRightPlaceMoveCommand(), moveToPlace, new InstantCommand(m_intake::openIntake, m_intake));
            case "middle":
                return autoAlign;
            default:
                return new SequentialCommandGroup(autoAlign, setPipelineToTags, trackApriltag, moveIntoPlace, moveToPlace, new InstantCommand(m_intake::openIntake, m_intake));
        }
    }*/
}