package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Intake extends SubsystemBase {

    private final TalonFX intakeMotor;
    private final TalonFX beltMotor;

    PneumaticHub m_PH = new PneumaticHub(2);
    private static int leftIntakeForwardChannel = 6;
    private static int leftIntakeReverseChannel = 7;

    private static int rightIntakeForwardChannel = 2;
    private static int rightIntakeReverseChannel = 3;

    DoubleSolenoid leftIntakDoubleSolenoid;
    DoubleSolenoid rightIntakeDoubleSolenoid;

    private final VelocityVoltage m_intakeVeloityRequest = new VelocityVoltage(0).withEnableFOC(true);
    private final VelocityVoltage m_beltVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);

    private final NeutralOut m_intakeNeutralOut = new NeutralOut();
    private final NeutralOut m_beltNeutralOut = new NeutralOut();

public Intake() {
    intakeMotor = new TalonFX(31, "TheCan");
    beltMotor = new TalonFX(21, "TheCan");


    leftIntakDoubleSolenoid = m_PH.makeDoubleSolenoid(leftIntakeForwardChannel, leftIntakeReverseChannel);
    rightIntakeDoubleSolenoid = m_PH.makeDoubleSolenoid(rightIntakeForwardChannel, rightIntakeReverseChannel);

    rightIntakeDoubleSolenoid.set(Value.kReverse);
    leftIntakDoubleSolenoid.set(Value.kReverse);
    
    configureIntakeMotor(intakeMotor);
    configureBeltMotor(beltMotor);

    SmartDashboard.putNumber("Hopper/SetIntakeRps", 90);
    SmartDashboard.putNumber("Hopper/SetIntakeClearRps", -75);
     SmartDashboard.putNumber("Hopper/SetBeltRps", 40);
}

public void configureIntakeMotor(TalonFX motor) {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Voltage.PeakForwardVoltage = 11.0;
    config.Voltage.PeakReverseVoltage = -11.0;

    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;


    config.Slot0.kV = 0.105; // 0.128;
    config.Slot0.kP = 0.13; // 0.15;
    config.Slot0.kD = 0.0;


    motor.getConfigurator().apply(config);
}

public void configureBeltMotor(TalonFX motor) {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Voltage.PeakForwardVoltage = 11.0;
    config.Voltage.PeakReverseVoltage = -11.0;

    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;


    config.Slot0.kV = 0.128;
    config.Slot0.kP = 0.1;
    config.Slot0.kD = 0.0;


    motor.getConfigurator().apply(config);
}

public void extendHopper() {
    rightIntakeDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
    leftIntakDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
    
}

public void retractHopper() {
    rightIntakeDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    leftIntakDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    
}

// public void startIntake() {
//     intakeMotor.set(-1.0);
// }

// public void stopIntake() {
//     intakeMotor.set(0.0);
// // }

// public void startBelt() {
//     beltMotor.set(0.3);
// }

// public void reverseBelt() {
//     beltMotor.set(-0.6);
// }

// public void stopBelt() {
//     beltMotor.set(0.0);
// }


public void runIntake(double rps) {
    intakeMotor.setControl(m_intakeVeloityRequest.withVelocity(rps));
}

public void stopIntake() {
    intakeMotor.setControl(m_intakeNeutralOut);
} 

public void runBelt(double rps) {
    beltMotor.setControl(m_beltVelocityRequest.withVelocity(rps));
}

public void stopBelt() {
    beltMotor.setControl(m_beltNeutralOut);
} 


@Override
    public void periodic() {


        double intakeRps = intakeMotor.getVelocity().getValueAsDouble();
        double beltRps = beltMotor.getVelocity().getValueAsDouble();

        SmartDashboard.putNumber("Hopper/IntakeRps", intakeRps);
        SmartDashboard.putNumber("Hopper/IntakeRpm", intakeRps * 60);

        SmartDashboard.putNumber("Hopper/BeltRps", beltRps);
        SmartDashboard.putNumber("Hopper/BeltRpm", beltRps * 60);
    }


    public Command RunIntakeCmd() {
        return new StartEndCommand(
            () -> {
                double rps = SmartDashboard.getNumber("Hopper/SetIntakeRps", 0.0);
                runIntake(rps);
            }, 
            () -> stopIntake());
    }

    public Command RunBeltCmd() {
        return new StartEndCommand(
             () -> {
                double rps = SmartDashboard.getNumber("Hopper/SetBeltRps", 0.0);
                runBelt(rps);
            }, 
            () -> stopBelt());
    }

    public Command RunClearBeltCmd() {
        return new WaitCommand(.25)
            .andThen(() -> runBelt(-40));
    }


    public Command RunClearIntakeCmd() {
        return new WaitCommand(0.25)
        .andThen(() -> { 
            double rps = SmartDashboard.getNumber("Hopper/SetIntakeClearRps", 0.0);
            runIntake(rps); 
        })
        .andThen(new WaitCommand(2.5)
            .andThen(() -> stopIntake()))
        .handleInterrupt(() -> stopIntake());
    }

}