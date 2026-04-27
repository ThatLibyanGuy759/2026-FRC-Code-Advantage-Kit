package frc.robot.hopper;

import java.util.concurrent.locks.Condition;

import org.littletonrobotics.junction.inputs.LoggableInputs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class HopperSubsystem extends SubsystemBase {

  private final TalonFX hopperMotor;
  private final HopperConfigs configs;
  private final VoltageOut voltageOut = new VoltageOut(0); 
  private final MotionMagicExpoVoltage setpointRequest = new MotionMagicExpoVoltage(0);

    private final HopperIO io;
    private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  public HopperSubsystem(HopperIO io) {  
    configs = new HopperConfigs();
    hopperMotor = new TalonFX(HopperConstants.hopperMotorID);
    this.io = io; 
    //we do this because just in case the configs dont get applied the first time
    for (int i = 0; i < 5; i++) {
      var status = hopperMotor.getConfigurator().apply(configs.hopperMotorConfig());
      if (status.isOK()) break;
    }
  }

  public void runHopperFront(double speed){
    hopperMotor.setControl(voltageOut.withOutput(speed));
    io.runVoltage(speed);
  }

  public void runHopperBack(double speed){
    hopperMotor.setControl(voltageOut.withOutput(-speed));
    io.runVoltage(-speed);
  }

  public double getPositionMeters() {
    double mechanismRotations = hopperMotor.getPosition(true).getValueAsDouble();
    return mechanismRotations * HopperConstants.metersPerRotation;
  }

  public void goToPosition(double meters) {
    goToPosition(meters, HopperConstants.expoKV);
    io.goToPosition(meters, HopperConstants.expoKV);
  }

  // expoKV controls peak speed: lower = faster (peak vel ≈ 12V / expoKV).
  public void goToPosition(double meters, double expoKV) {
    MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
    mmConfigs.MotionMagicExpo_kV = expoKV;
    mmConfigs.MotionMagicExpo_kA = HopperConstants.expoKA;
    hopperMotor.getConfigurator().apply(mmConfigs);
    hopperMotor.getConfigurator().apply(configs.hopperMotorConfig());
    double targetRotations = meters / HopperConstants.metersPerRotation;
    hopperMotor.setControl(setpointRequest.withPosition(targetRotations));
    io.goToPosition(meters, expoKV);
  }

  public void zeroHopper() {
    hopperMotor.setPosition(0.0);
    io.zeroPosition();
  }

  public void applyIdleConfigs() {
    hopperMotor.getConfigurator().apply(configs.idleHopperMotorConfig());
    io.applyIdleConfigs();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hopper Position Meters", getPositionMeters());
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
 
    Logger.recordOutput("Hopper/MotorConnected", inputs.motorConnected);
  }

  @Override
  public void simulationPeriodic() {
  }
}