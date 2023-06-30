package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
    private CANSparkMax shootMotor;
    private CANSparkMax rotateMotor;
    private CANSparkMax intakeMotor;

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

    }
}
