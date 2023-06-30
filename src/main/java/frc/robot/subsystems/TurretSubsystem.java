package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
    private final CANSparkMax shootMotor;
    private final CANSparkMax rotateMotor;
    private final CANSparkMax intakeMotor;

    private final RelativeEncoder shootEncoder;
    private final RelativeEncoder rotateEncoder;
    private final RelativeEncoder intakeEncoder;

    private final SparkMaxPIDController shootPIDController;
    private final SparkMaxPIDController rotatePIDController;
    private final SparkMaxPIDController intakePIDController;

    public TurretSubsystem() {
        shootMotor = new CANSparkMax(1, CANSparkMax.MotorType.kBrushless);
        rotateMotor = new CANSparkMax(2, CANSparkMax.MotorType.kBrushless);
        intakeMotor = new CANSparkMax(3, CANSparkMax.MotorType.kBrushless);

        shootMotor.restoreFactoryDefaults();
        rotateMotor.restoreFactoryDefaults();
        intakeMotor.restoreFactoryDefaults();

        shootMotor.setInverted(false); // TODO: Check if this is correct
        rotateMotor.setInverted(false); // TODO: Check if this is correct
        intakeMotor.setInverted(false); // TODO: Check if this is correct

        shootMotor.setSmartCurrentLimit(TurretConstants.kShootMotorCurrentLimit);
        rotateMotor.setSmartCurrentLimit(TurretConstants.kRotateMotorCurrentLimit);
        intakeMotor.setSmartCurrentLimit(TurretConstants.kIntakeMotorCurrentLimit);

        shootMotor.setIdleMode(TurretConstants.kShootMotorIdleMode);
        rotateMotor.setIdleMode(TurretConstants.kRotateMotorIdleMode);
        intakeMotor.setIdleMode(TurretConstants.kIntakeMotorIdleMode);

        shootEncoder = shootMotor.getEncoder();
        rotateEncoder = rotateMotor.getEncoder();
        intakeEncoder = intakeMotor.getEncoder();

        shootPIDController = shootMotor.getPIDController();
        rotatePIDController = rotateMotor.getPIDController();
        intakePIDController = intakeMotor.getPIDController();
        shootPIDController.setFeedbackDevice(shootEncoder);
        rotatePIDController.setFeedbackDevice(rotateEncoder);
        intakePIDController.setFeedbackDevice(intakeEncoder);

    }
}
