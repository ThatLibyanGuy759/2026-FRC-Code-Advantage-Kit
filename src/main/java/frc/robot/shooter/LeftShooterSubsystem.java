// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
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

public class LeftShooterSubsystem extends SubsystemBase {
    private final TalonFX shooterMotorOne;
    private final TalonFX shooterMotorTwo;
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0); 
    private final VelocityVoltage voltageRequest = new VelocityVoltage(0).withEnableFOC(true);
    private final VoltageOut sysIdControl = new VoltageOut(0);
    private final SysIdRoutine m_SysIdRoutine;
    private final LeftShooterConfigs configs;
    private final StatusSignal<AngularVelocity> leftShooterVelocitySignal;
    private final StatusSignal<Voltage> leftShootervoltageSignal;
    private final StatusSignal<Current> leftShooterCurrentSignal;
    private final TalonFXSimState motorOneSimState;
    private final TalonFXSimState motorTwoSimState;
    private final FlywheelSim flywheelSim;

    private double lastKP = ShooterConstants.leftKP;
    private double lastKI = ShooterConstants.leftKI;
    private double lastKD = ShooterConstants.leftKD;
    private double lastKS = ShooterConstants.leftKS;
    private double lastKV = ShooterConstants.leftKV;

    /** Creates a new ExampleSubsystem. */
  public LeftShooterSubsystem() {

    shooterMotorOne = new TalonFX(ShooterConstants.leftShooterMotorOneID);
    shooterMotorTwo = new TalonFX(ShooterConstants.leftShooterMotorTwoID);
    motorOneSimState = shooterMotorOne.getSimState();
    motorTwoSimState = shooterMotorTwo.getSimState();
    flywheelSim = new FlywheelSim(
    LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.003, 1.0), DCMotor.getKrakenX60Foc(1));

    configs = new LeftShooterConfigs();
      m_SysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null,
      Volts.of(4),
      null, 
      (state) -> SignalLogger.writeString("state", state.toString())),
      new SysIdRoutine.Mechanism(
        (volts) ->{
          shooterMotorOne.setControl(sysIdControl.withOutput(volts.in(Volts)));
          shooterMotorTwo.setControl(sysIdControl.withOutput(volts.in(Volts)));
        },

        log -> {
          log.motor("Left Shooter Motor One")
          .voltage(shooterMotorOne.getMotorVoltage().getValue())
          .angularPosition(shooterMotorOne.getPosition().getValue())
          .angularVelocity(shooterMotorOne.getVelocity().getValue());

          log.motor("Left Shooter Motor Two")
          .voltage(shooterMotorTwo.getMotorVoltage().getValue())
          .angularPosition(shooterMotorTwo.getPosition().getValue())
          .angularVelocity(shooterMotorTwo.getVelocity().getValue());
        },
      this));
    configureMotors();
    SignalLogger.setPath("/home/vuser/logs/");
    
    leftShooterCurrentSignal = shooterMotorOne.getSupplyCurrent();
    leftShooterVelocitySignal = shooterMotorOne.getVelocity();
    leftShootervoltageSignal = shooterMotorOne.getMotorVoltage();

    SmartDashboard.putNumber("Left Shooter kP", ShooterConstants.leftKP);
    SmartDashboard.putNumber("Left Shooter kI", ShooterConstants.leftKI);
    SmartDashboard.putNumber("Left Shooter kD", ShooterConstants.leftKD);
    SmartDashboard.putNumber("Left Shooter kS", ShooterConstants.leftKS);
    SmartDashboard.putNumber("Left Shooter kV", ShooterConstants.leftKV);
    
    
  }
  // 44 inch from hub
  public void configureMotors() {

    for (int i = 0; i < 5; i++) {
      var status = shooterMotorOne.getConfigurator().apply(configs.shooterMotorConfig());
      if (status.isOK()) break;
    }

    for (int i = 0; i < 5; i++) {
      var status = shooterMotorTwo.getConfigurator().apply(configs.shooterMotorConfig());
      if (status.isOK()) break;
    }
    

    shooterMotorOne.getVelocity().setUpdateFrequency(100);
    shooterMotorTwo.getVelocity().setUpdateFrequency(100);
    shooterMotorOne.getPosition().setUpdateFrequency(100);
    shooterMotorTwo.getPosition().setUpdateFrequency(100);

  }
  public Command sysIdDynamic(SysIdRoutine.Direction direction){
    return m_SysIdRoutine.dynamic(direction);
  }
  
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
    return m_SysIdRoutine.quasistatic(direction);
  }

  public void shootFuel(double speed) {
    shooterMotorOne.setControl(dutyCycle.withOutput(speed));
  }

  public void runVelocity(double rps) {
    voltageRequest.Velocity = rps;
    shooterMotorOne.setControl(voltageRequest); 
  }

  private final VelocityTorqueCurrentFOC torqueFocRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);  

  public void runVelocityTorqueFOC(double rps) {
    double motorRPS = rps; 
    double actualRPS = shooterMotorOne.getVelocity().refresh().getValueAsDouble();

    shooterMotorOne.setControl(torqueFocRequest.withVelocity(rps));
    shooterMotorTwo.setControl(torqueFocRequest.withVelocity(rps));

    SmartDashboard.putNumber("LEFT Motor Target RPS", motorRPS);
    SmartDashboard.putNumber("LEFT Motor Actual RPS", actualRPS);
  }


  public void setVoltage(double voltage){
    shooterMotorOne.setVoltage(voltage);
    shooterMotorTwo.setVoltage(voltage);
  }
  
  public void setRawVbus(){
    var voltageRequest = new VoltageOut(0);

    shooterMotorOne.setControl(voltageRequest.withOutput(12));

    printLeftRPM();
  }

  public void printVoltageOutput() {
    double motorVoltage = shooterMotorOne.getMotorVoltage().getValueAsDouble();
    SmartDashboard.putNumber("Motor Voltage", motorVoltage);
  }

  public void resetVoltageOutput() {
    SmartDashboard.putNumber("Motor Voltage", 0);
  }

  public void printCurrentLimits() {
    SmartDashboard.putNumber("Shooter Stator Current", shooterMotorOne.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Supply Current", shooterMotorOne.getSupplyCurrent().getValueAsDouble());
  }

  public void printLeftRPM() {
    double motorRPS = shooterMotorOne.getVelocity().getValueAsDouble();
    double shooterRPM = motorRPS * 60.0;
    SmartDashboard.putNumber("Shooter Motor RPM", shooterRPM);
  }


  @Override
  public void periodic() {
    double newKP = SmartDashboard.getNumber("Left Shooter kP", ShooterConstants.leftKP);
    double newKI = SmartDashboard.getNumber("Left Shooter kI", ShooterConstants.leftKI);
    double newKD = SmartDashboard.getNumber("Left Shooter kD", ShooterConstants.leftKD);
    double newKS = SmartDashboard.getNumber("Left Shooter kS", ShooterConstants.leftKS);
    double newKV = SmartDashboard.getNumber("Left Shooter kV", ShooterConstants.leftKV);

    if (newKP != lastKP || newKI != lastKI || newKD != lastKD || newKS != lastKS || newKV != lastKV){
      ShooterConstants.leftKP = newKP;
      ShooterConstants.leftKI = newKI;
      ShooterConstants.leftKD = newKD;
      ShooterConstants.leftKS = newKS;
      ShooterConstants.leftKV = newKV;
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