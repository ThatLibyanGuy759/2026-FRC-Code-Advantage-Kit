package frc.robot.hopper;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class HopperSubsystem extends SubsystemBase {
  
  private final TalonFX hopperMotor;
  private final HopperConfigs configs;
  private final VoltageOut voltageOut = new VoltageOut(0); 
  private final MotionMagicExpoVoltage setpointRequest = new MotionMagicExpoVoltage(0);

  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();


  private final TalonFXSimState hopperMotorSim;
  private final ElevatorSim hopperSim;

  private final StructArrayPublisher<Pose3d> hopperPosePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("SmartDashboard/HopperPoses", Pose3d.struct).publish();

  public HopperSubsystem(HopperIO io) {
    configs = new HopperConfigs();
    hopperMotor = new TalonFX(HopperConstants.hopperMotorID);

    this.io = io;
    if (io instanceof HopperIOSim) {
      hopperMotorSim = ((HopperIOSim) io).getSimState();
      hopperSim = ((HopperIOSim) io).getElevatorSim();
    } else {
      hopperMotorSim = null;
      hopperSim = null;
    }

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
    return inputs.positionMeters;
  }

  public void goToPosition(double meters) {
    goToPosition(meters, HopperConstants.expoKV);
    io.goToPosition(meters, HopperConstants.expoKV);
  }

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
    Logger.recordOutput("Hopper/PositionMeters", inputs.positionMeters);
  }

  @Override
  public void simulationPeriodic() {
    if (hopperMotorSim == null || hopperSim == null) return;

    Logger.recordOutput("Hopper/SimPositionMeters", hopperSim.getPositionMeters());
    Logger.recordOutput("Hopper/SimVelocityMetersPerSec", hopperSim.getVelocityMetersPerSecond());

    double hopperPose = hopperSim.getPositionMeters();
    hopperPosePublisher.set(new Pose3d[] {
        new Pose3d(new Translation3d(hopperPose, -0.01, 0.0), 
        new Rotation3d()
        )
      }
    );
  }
}