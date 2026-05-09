package frc.robot.hopper;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class HopperIOSim implements HopperIO {

    private final TalonFX motor;
    private final TalonFXSimState simState;
    private final ElevatorSim hopperSim;


    private final ProfiledPIDController controller = new ProfiledPIDController(50.0, 0.0, 0.0, new TrapezoidProfile.Constraints(12.0 / HopperConstants.expoKV,12.0 / HopperConstants.expoKA));
    private double appliedVolts = 0.0;
    private boolean closedLoop = false;

    public HopperIOSim() {
        motor = new TalonFX(HopperConstants.hopperMotorID);
        simState = motor.getSimState();

        hopperSim = new ElevatorSim(
            DCMotor.getKrakenX60(1),
            HopperConstants.gearRatio,
            HopperConstants.carriageMassKg,
            HopperConstants.drumRadiusMeters,
            0.0,
            HopperConstants.maxExtensionMeters,
            false,
            0.0
        );
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        if (closedLoop) {
            appliedVolts = controller.calculate(hopperSim.getPositionMeters());
            appliedVolts = Math.max(-12.0, Math.min(12.0, appliedVolts));
        }

        hopperSim.setInputVoltage(appliedVolts);
        hopperSim.update(0.020);

        double simPositionRot = hopperSim.getPositionMeters() / HopperConstants.metersPerRotation;
        double simVelocityRPS = hopperSim.getVelocityMetersPerSecond() / HopperConstants.metersPerRotation;
        simState.setRawRotorPosition(simPositionRot);
        simState.setRotorVelocity(simVelocityRPS);

        inputs.positionMeters = hopperSim.getPositionMeters();
        inputs.velocityMetersPerSec = hopperSim.getVelocityMetersPerSecond();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = hopperSim.getCurrentDrawAmps();
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
        hopperSim.setState(0.0, 0.0);
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
        return hopperSim;
    }
}