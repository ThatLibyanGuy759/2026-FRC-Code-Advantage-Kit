package frc.robot.elevator;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {

    private final TalonFX motor;
    private final TalonFXSimState simState;
    private final ElevatorSim elevatorSim;
    private final ElevatorConfigs configs;

    private final ProfiledPIDController controller = new ProfiledPIDController(50.0, 0.0, 0.0, new TrapezoidProfile.Constraints(12.0 / ElevatorConstants.expoKV, 12.0 / ElevatorConstants.expoKA));
    private double appliedVolts = 0.0;
    private boolean closedLoop = false;

    public ElevatorIOSim() {
        motor = new TalonFX(ElevatorConstants.elevatorMotorID);
        simState = motor.getSimState();
        configs = new ElevatorConfigs();

        for (int i = 0; i < 5; i++) {
            var status = motor.getConfigurator().apply(configs.elevatorMotorConfig());
            if (status.isOK()) break;
        }

        elevatorSim = new ElevatorSim(
            DCMotor.getKrakenX60(1),
            ElevatorConstants.gearRatio,
            2.0, // dud numbers
            ElevatorConstants.pulleyCircumfrence,
            0.0,
            ElevatorConstants.forwardSoftLimitMeters, 
            true,
            0.0
        );
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        if (closedLoop) {
            appliedVolts = controller.calculate(elevatorSim.getPositionMeters());
            appliedVolts = Math.max(-12.0, Math.min(12.0, appliedVolts));
        }

        elevatorSim.setInputVoltage(appliedVolts);
        elevatorSim.update(0.020);

        double simPositionRot = elevatorSim.getPositionMeters() / ElevatorConstants.metersPerRotation;
        double simVelocityRPS = elevatorSim.getVelocityMetersPerSecond() / ElevatorConstants.metersPerRotation;
        simState.setRawRotorPosition(simPositionRot);
        simState.setRotorVelocity(simVelocityRPS);

        inputs.positionMeters = elevatorSim.getPositionMeters();
        inputs.velocityMetersPerSec = elevatorSim.getVelocityMetersPerSecond();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = elevatorSim.getCurrentDrawAmps();
        inputs.motorConnected = true;
    }

    @Override
    public void runVoltage(double volts) {
        closedLoop = false;
        appliedVolts = volts;
    }

    @Override
    public void goToPosition(double meters, double expoKV) {
        closedLoop = true;
        controller.setGoal(meters);
    }

    @Override
    public void zeroPosition() {
        elevatorSim.setState(0.0, 0.0);
        controller.reset(0.0);
    }

    @Override
    public void applyIdleConfigs() {
        closedLoop = false;
        appliedVolts = 0.0;
    }

    public TalonFXSimState getSimState() {
        return simState;
    }

    public ElevatorSim getElevatorSim() {
        return elevatorSim;
    }
}
