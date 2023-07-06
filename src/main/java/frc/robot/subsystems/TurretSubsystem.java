package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
    private LimeLight limeLight;

    private final CANSparkMax shootMotor;
    private final CANSparkMax rotateMotor;
    private final CANSparkMax intakeMotor;

    private final RelativeEncoder shootEncoder;
    private final RelativeEncoder rotateEncoder;
    private final RelativeEncoder intakeEncoder;

    public TurretSubsystem() {
        shootMotor = new CANSparkMax(Constants.TurretConstants.kShootMotorCanID, CANSparkMax.MotorType.kBrushless);
        rotateMotor = new CANSparkMax(Constants.TurretConstants.kRotateMotorCanID, CANSparkMax.MotorType.kBrushless);
        intakeMotor = new CANSparkMax(Constants.TurretConstants.kIntakeMotorCanID, CANSparkMax.MotorType.kBrushless);

        shootMotor.restoreFactoryDefaults();
        rotateMotor.restoreFactoryDefaults();
        intakeMotor.restoreFactoryDefaults();

        shootMotor.setInverted(true); 
        rotateMotor.setInverted(false); // Personal preference
        intakeMotor.setInverted(false);

        shootMotor.setSmartCurrentLimit(TurretConstants.kShootMotorCurrentLimit);
        rotateMotor.setSmartCurrentLimit(TurretConstants.kRotateMotorCurrentLimit);
        intakeMotor.setSmartCurrentLimit(TurretConstants.kIntakeMotorCurrentLimit);

        shootMotor.setIdleMode(TurretConstants.kShootMotorIdleMode);
        rotateMotor.setIdleMode(TurretConstants.kRotateMotorIdleMode);
        intakeMotor.setIdleMode(TurretConstants.kIntakeMotorIdleMode);

        shootEncoder = shootMotor.getEncoder();
        rotateEncoder = rotateMotor.getEncoder();
        intakeEncoder = intakeMotor.getEncoder();

        PIDController turretPID = new PIDController(TurretConstants.turnP, 
                                                    TurretConstants.turnI, 
                                                    TurretConstants.turnD);
    }


    public void teleopTurret(Double rotate, Boolean shoot){
        rotateMotor.set(rotate);
        if(shoot){
            shootMotor.set(1);
            intakeMotor.set(1);
        } else {
            shootMotor.set(0);
        }
    }

    public void autoTurret(){
    }
}
