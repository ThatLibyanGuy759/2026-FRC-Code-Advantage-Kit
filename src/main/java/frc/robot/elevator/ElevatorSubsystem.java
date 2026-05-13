package frc.robot.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
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

public class ElevatorSubsystem extends SubsystemBase {

  private final TalonFX elevatorMotor;
  private final ElevatorConfigs configs;
  private final MotionMagicExpoVoltage setpointRequest = new MotionMagicExpoVoltage(0);

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final TalonFXSimState elevatorMotorSim;
  private final ElevatorSim elevatorSim;

  private final StructArrayPublisher<Pose3d> elevatorPosePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("SmartDashboard/ElevatorPoses", Pose3d.struct).publish();

  public ElevatorSubsystem(ElevatorIO io) {
    configs = new ElevatorConfigs();
    elevatorMotor = new TalonFX(ElevatorConstants.elevatorMotorID);

    this.io = io;
    if (io instanceof ElevatorIOSim) {
      elevatorMotorSim = ((ElevatorIOSim) io).getSimState();
      elevatorSim = ((ElevatorIOSim) io).getElevatorSim();
    } else {
      elevatorMotorSim = null;
      elevatorSim = null;
    }

    //we do this because just in case the configs dont get applied the first time
    for (int i = 0; i < 5; i++) {
      var status = elevatorMotor.getConfigurator().apply(configs.elevatorMotorConfig());
      if (status.isOK()) break;
    }
  }

  public void runelevatorFront(double speed){
    io.runVoltage(speed);
  }

  public void runelevatorBack(double speed){
    io.runVoltage(-speed);
  }

  public double getPositionMeters() {
    return inputs.positionMeters;
  }

  public void goToPosition(double meters) {
    goToPosition(meters, ElevatorConstants.expoKV);
  }

  // expoKV controls peak speed: lower = faster (peak vel ≈ 12V / expoKV).
  public void goToPosition(double meters, double expoKV) {
    MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
    mmConfigs.MotionMagicExpo_kV = expoKV;
    mmConfigs.MotionMagicExpo_kA = ElevatorConstants.expoKA;
    elevatorMotor.getConfigurator().apply(mmConfigs);
    double targetRotations = meters / ElevatorConstants.metersPerRotation;
    elevatorMotor.setControl(setpointRequest.withPosition(targetRotations));
    io.goToPosition(meters, expoKV);
  }

  public void zeroelevator() {
    elevatorMotor.setPosition(0.0);
    io.zeroPosition();
  }

  public void applyIdleConfigs() {
    elevatorMotor.getConfigurator().apply(configs.idleelevatorMotorConfig());
    io.applyIdleConfigs();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position Meters", getPositionMeters());
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/MotorConnected", inputs.motorConnected);
    Logger.recordOutput("Elevator/PositionMeters", inputs.positionMeters);
  }

  @Override
  public void simulationPeriodic() {
    if (elevatorMotorSim == null || elevatorSim == null) return;

    Logger.recordOutput("Elevator/SimPositionMeters", elevatorSim.getPositionMeters());
    Logger.recordOutput("Elevator/SimVelocityMetersPerSec", elevatorSim.getVelocityMetersPerSecond());

    double elevatorHeight = elevatorSim.getPositionMeters();
    elevatorPosePublisher.set(new Pose3d[] {
        new Pose3d(new Translation3d(0.0, 0.0, elevatorHeight), 
        new Rotation3d()
        )
      }
    );
  }
}