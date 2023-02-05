package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClawSubsystem extends SubsystemBase {

    private final CANSparkMax intakeMotor;
    private final DoubleSolenoid clawSolenoid;

    public ClawSubsystem() {
        intakeMotor = new CANSparkMax(Constants.Claw.INTAKE_MOTOR, MotorType.kBrushless);
        clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Claw.SOLINOID_FORWARD, Constants.Claw.SOLINOID_REVERSE);
    }

    public void setIntakeMotor(double speed) {
        intakeMotor.set(speed);
    }

    public void stopIntakeMotor() {
        intakeMotor.set(0);
    }

    public void openClaw() {
        clawSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void closeClaw() {
        clawSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
}