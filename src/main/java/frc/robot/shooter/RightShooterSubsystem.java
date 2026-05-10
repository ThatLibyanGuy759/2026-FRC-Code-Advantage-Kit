// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.system.plant.LinearSystemId;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.controls.VoltageOut;

public class RightShooterSubsystem extends SubsystemBase {
    private final TalonFX shooterMotorThree;
    private final TalonFX shooterMotorFour;
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0); 
    private final VelocityVoltage voltageRequest = new VelocityVoltage(0).withEnableFOC(true);
    private final VoltageOut sysIdControl = new VoltageOut(0);
    private final SysIdRoutine m_SysIdRoutine;
    private final RightShooterConfigs configs;
    private final TalonFXSimState motorOneSimState;
    private final TalonFXSimState motorTwoSimState;
    private final FlywheelSim flywheelSim;

    private double lastKP = ShooterConstants.rightKP;
    private double lastKI = ShooterConstants.rightKI;
    private double lastKD = ShooterConstants.rightKD;
    private double lastKS = ShooterConstants.rightKS;
    private double lastKV = ShooterConstants.rightKV;

    /** Creates a new ExampleSubsystem. */
  public RightShooterSubsystem() {

    shooterMotorThree = new TalonFX(ShooterConstants.rightShooterMotorOneID);
    shooterMotorFour = new TalonFX(ShooterConstants.rightShooterMotorTwoID);
    configs = new RightShooterConfigs();
    motorOneSimState = shooterMotorThree.getSimState();
    motorTwoSimState = shooterMotorFour.getSimState();
    flywheelSim = new FlywheelSim(
    LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.003, 1.0), DCMotor.getKrakenX60Foc(1));
    m_SysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null,
      Volts.of(4),
      null, 
      (state) -> SignalLogger.writeString("state", state.toString())),
      new SysIdRoutine.Mechanism(
        (volts) -> {
          shooterMotorThree.setControl(sysIdControl.withOutput(volts.in(Volts)));
          shooterMotorFour.setControl(sysIdControl.withOutput(volts.in(Volts)));
        },

        log -> {
          log.motor("Right Shooter Motor One")
          .voltage(shooterMotorThree.getMotorVoltage().getValue())
          .angularPosition(shooterMotorThree.getPosition().getValue())
          .angularVelocity(shooterMotorThree.getVelocity().getValue());

          log.motor("Right Shooter Motor Two")
          .voltage(shooterMotorFour.getMotorVoltage().getValue())
          .angularPosition(shooterMotorFour.getPosition().getValue())
          .angularVelocity(shooterMotorFour.getVelocity().getValue());
        },
      this));
    configureMotors();
    SignalLogger.setPath("/home/vuser/logs/");

    SmartDashboard.putNumber("Right Shooter kP", ShooterConstants.rightKP);
    SmartDashboard.putNumber("Right Shooter kI", ShooterConstants.rightKI);
    SmartDashboard.putNumber("Right Shooter kD", ShooterConstants.rightKD);
    SmartDashboard.putNumber("Right Shooter kS", ShooterConstants.rightKS);
    SmartDashboard.putNumber("Right Shooter kV", ShooterConstants.rightKV);
    
  }
  // 44 inch from hub
  public void configureMotors() {

    for (int i = 0; i < 5; i++) {
      var status = shooterMotorThree.getConfigurator().apply(configs.shooterMotorConfig());
      if (status.isOK()) break;
    }

    for (int i = 0; i < 5; i++) {
      var status = shooterMotorFour.getConfigurator().apply(configs.shooterMotorConfig());
      if (status.isOK()) break;
    }

    shooterMotorThree.getVelocity().setUpdateFrequency(100);
    shooterMotorFour.getVelocity().setUpdateFrequency(100);
    shooterMotorThree.getPosition().setUpdateFrequency(100);
    shooterMotorFour.getPosition().setUpdateFrequency(100);

  }
  public Command sysIdDynamic(SysIdRoutine.Direction direction){
    return m_SysIdRoutine.dynamic(direction);
  }
  
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
    return m_SysIdRoutine.quasistatic(direction);
  }

  public void shootFuel(double speed) {
    shooterMotorThree.setControl(dutyCycle.withOutput(speed));
  }

  public void runVelocity(double rps) {
    voltageRequest.Velocity = rps;
    shooterMotorThree.setControl(voltageRequest); 
  }
  
  private final VelocityTorqueCurrentFOC torqueFocRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);  

  public void runVelocityTorqueFOC(double rps) {
      double motorRPS = -rps; 
      double actualRPS = shooterMotorThree.getVelocity().refresh().getValueAsDouble();


      shooterMotorThree.setControl(torqueFocRequest.withVelocity(motorRPS));
      shooterMotorFour.setControl(torqueFocRequest.withVelocity(motorRPS));

      
      SmartDashboard.putNumber("RIGHT Motor Target RPS", motorRPS);
      SmartDashboard.putNumber("RIGHT Motor Actual RPS", actualRPS);
  }


  public void setVoltage(double voltage) {
    shooterMotorThree.setVoltage(voltage);
    shooterMotorFour.setVoltage(voltage);
  }
  
  public void setRawVbus() {
    var voltageRequest = new VoltageOut(0);

    shooterMotorThree.setControl(voltageRequest.withOutput(12));

    printRightRPM();
  }

  public void printVoltageOutput() {
    double motorVoltage = shooterMotorThree.getMotorVoltage().getValueAsDouble();
    SmartDashboard.putNumber("Motor Voltage", motorVoltage);
  }

  public void resetVoltageOutput() {
    SmartDashboard.putNumber("Motor Voltage", 0);
  }

  public void printCurrentLimits() {
    SmartDashboard.putNumber("Shooter Stator Current", shooterMotorThree.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Supply Current", shooterMotorThree.getSupplyCurrent().getValueAsDouble());
  }

  public void printRightRPM() {
    double motorRPS = shooterMotorThree.getVelocity().getValueAsDouble();
    double shooterRPM = motorRPS * 60.0;
    SmartDashboard.putNumber("Right Shooter RPM", Math.abs(shooterRPM));
  }

  @Override
  public void periodic() {
    double newKP = SmartDashboard.getNumber("Right Shooter kP", ShooterConstants.rightKP);
    double newKI = SmartDashboard.getNumber("Right Shooter kI", ShooterConstants.rightKI);
    double newKD = SmartDashboard.getNumber("Right Shooter kD", ShooterConstants.rightKD);
    double newKS = SmartDashboard.getNumber("Right Shooter kS", ShooterConstants.rightKS);
    double newKV = SmartDashboard.getNumber("Right Shooter kV", ShooterConstants.rightKV);
    printRightRPM();
    if (newKP != lastKP || newKI != lastKI || newKD != lastKD || newKS != lastKS || newKV != lastKV){
      ShooterConstants.rightKP = newKP;
      ShooterConstants.rightKI = newKI;
      ShooterConstants.rightKD = newKD;
      ShooterConstants.rightKS = newKS;
      ShooterConstants.rightKV = newKV;
      configureMotors();
      lastKP = newKP;
      lastKI = newKI;
      lastKD = newKD;
      lastKS = newKS;
      lastKV = newKV;
    }
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    motorOneSimState.setSupplyVoltage(RoboRioSim.getVInVoltage());
    motorTwoSimState.setSupplyVoltage(RoboRioSim.getVInVoltage());

    double motorVoltage = motorOneSimState.getMotorVoltage();
    flywheelSim.setInputVoltage(motorVoltage);
    flywheelSim.update(0.020);

    double simRPS = flywheelSim.getAngularVelocityRPM() / 60.0;
    motorOneSimState.setRotorVelocity(simRPS);
    motorOneSimState.addRotorPosition(simRPS * 0.020);
    motorTwoSimState.setRotorVelocity(simRPS);
    motorTwoSimState.addRotorPosition(simRPS * 0.020);

    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));
  }
}