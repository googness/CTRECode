package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Value;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Alert;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Shooter extends SubsystemBase {
    
private final TalonFX frontShooter;
private final TalonFX backShooter;
private final TalonFX feederMotor;

    PneumaticHub m_PH = new PneumaticHub(2);
    private static int hoodForwardChannel = 4;
    private static int hoodReverseChannel = 5;

    DoubleSolenoid hoodDoubleSolenoid;



    private final VelocityVoltage m_frontVeloityRequest = new VelocityVoltage(0).withEnableFOC(true);
    private final VelocityVoltage m_backVeloityRequest = new VelocityVoltage(0).withEnableFOC(true);
    private final VelocityVoltage m_feederVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);

    private final NeutralOut m_frontNeutralOut = new NeutralOut();
    private final NeutralOut m_backNeutralOut = new NeutralOut();
    private final NeutralOut m_feedeNeutralOut = new NeutralOut();

public Shooter() {
    frontShooter = new TalonFX(23, "TheCan");
    backShooter = new TalonFX(24, "TheCan");
    feederMotor = new TalonFX(22, "TheCan");

    hoodDoubleSolenoid = m_PH.makeDoubleSolenoid(hoodForwardChannel, hoodReverseChannel);

    hoodDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);


    configureBackMotor(backShooter);
    configureFrontMotor(frontShooter);
    configureFeederMotor(feederMotor);

     SmartDashboard.putNumber("Shooter/SetFrontRps", 49);
     SmartDashboard.putNumber("Shooter/SetBackRps", 49);
     SmartDashboard.putNumber("Shooter/SetFeedRps", 50);
}

public void configureFrontMotor(TalonFX motor) {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Voltage.PeakForwardVoltage = 11.0;
    config.Voltage.PeakReverseVoltage = -11.0;

    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Slot0.kP = 0.13; // .11
    config.Slot0.kV = 0.130; // .117;
    config.Slot0.kS = 0.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kD = 0.0; 


    motor.getConfigurator().apply(config);
}

public void configureBackMotor(TalonFX motor) {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Voltage.PeakForwardVoltage = 11.0;
    config.Voltage.PeakReverseVoltage = -11.0;

    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Slot0.kP = 0.13; // .11
    config.Slot0.kV = 0.130; // .117;
    config.Slot0.kS = 0.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kD = 0.0; 


    motor.getConfigurator().apply(config);
}

public void configureFeederMotor(TalonFX motor) {

    TalonFXConfiguration config = new TalonFXConfiguration();

    
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Voltage.PeakForwardVoltage = 11.0;
    config.Voltage.PeakReverseVoltage = -11.0;

    config.Slot0.kP = 0.11; // .11
    config.Slot0.kV = 0.132; // .117;
    
    config.Slot0.kD = 0.0; 

    // config.CurrentLimits.StatorCurrentLimit = 60;
    // config.CurrentLimits.StatorCurrentLimitEnable = true;

    // config.CurrentLimits.SupplyCurrentLimit = 40;
    // config.CurrentLimits.SupplyCurrentLimitEnable = true;


    motor.getConfigurator().apply(config);
}


public void runFlywheel(double frps, double brps) {
    frontShooter.setControl(m_frontVeloityRequest.withVelocity(frps));
    backShooter.setControl(m_backVeloityRequest.withVelocity(brps));
}

public void stop() {
    frontShooter.setControl(m_frontNeutralOut);
    backShooter.setControl(m_backNeutralOut);
}

public void runFeeder(double rps) {
    feederMotor.setControl(m_feederVelocityRequest.withVelocity(rps));
}

public void stopFeeder() {
    feederMotor.setControl(m_feedeNeutralOut);
}



public void extendHood() {
    hoodDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
}

public void retractHood() {
    hoodDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
}



public void runFlywheel() {
    // frontShooter.set(.45);
    // backShooter.set(.7);
     frontShooter.set(1);
    backShooter.set(1);
}
public void stopFlywheel() {
    frontShooter.set(0);
    backShooter.set(0);
}



public void startFeed() {
    feederMotor.set(0.65);
}

public void stopFeed() {
    feederMotor.set(0.0);
}


public void spinAndFeed() {
    frontShooter.set(0.3);
    backShooter.set(0.3);
    feederMotor.set(0.3);
}

public void stopSpinandFeed() {
    frontShooter.set(0.0);
    backShooter.set(0.0);
    feederMotor.set(0.0);
}



    @Override
    public void periodic() {


       

        double frontShooterRps = frontShooter.getVelocity().getValueAsDouble();
        double backShooterRps = backShooter.getVelocity().getValueAsDouble();
        double feederMotorRps = feederMotor.getVelocity().getValueAsDouble();

        SmartDashboard.putNumber("Shooter/FrontRps", frontShooterRps);
        SmartDashboard.putNumber("Shooter/FrontRpm", frontShooterRps * 60);

        SmartDashboard.putNumber("Shooter/BackRps", backShooterRps);
        SmartDashboard.putNumber("Shooter/BackRpm", backShooterRps * 60);

        SmartDashboard.putNumber("Shooter/FeedRps", feederMotorRps);
        SmartDashboard.putNumber("Shooter/FeedRpm", feederMotorRps * 60);
    }

    public Command ShootRpsCmd() {
        return new StartEndCommand(
            () -> {
                double frps = SmartDashboard.getNumber("Shooter/SetFrontRps", 0.0);
                double brps = SmartDashboard.getNumber("Shooter/SetBackRps", 0.0);
                runFlywheel(frps,brps);
            }, 
            () -> stop());
    }

    public Command FeederRpsCmd() {
        return new StartEndCommand(
            () -> {
                double frps = SmartDashboard.getNumber("Shooter/SetFeedRps", 0.0);
                
                runFeeder(frps);
            }, 
            () -> stopFeeder());
    }

}
