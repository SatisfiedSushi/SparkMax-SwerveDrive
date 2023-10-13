package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final CANSparkMax intakeAcativationMotor = new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax intakeLeftGripMotor = new CANSparkMax(9, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax intakeRightGripMotor = new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);


    public void triggerInputHandler(double left, double right) {
        SmartDashboard.putNumber("intake", left + right);
        double max = Math.max(left, right);
        if (max == left) {
            max = -max;
        }
        intakeAcativationMotor.set(max);
    }

    public void closeIntake() {
        intakeLeftGripMotor.set(-0.1);
        intakeRightGripMotor.set(0.1);
    }

    public void openIntake() {
        intakeLeftGripMotor.set(0.1);
        intakeRightGripMotor.set(-0.1);
    }
}
