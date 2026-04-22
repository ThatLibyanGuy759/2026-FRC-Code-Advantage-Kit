package frc.robot.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class LeftShooterIOSim implements LeftShooterIO {

    private static final double GEAR_RATIO = 1.0; //dud numbers
    private static final double MOMENT_OF_INERTIA = 0.004; //dud numbers

    private final FlywheelSim sim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60(2),
            MOMENT_OF_INERTIA,
            GEAR_RATIO
        ),
        DCMotor.getKrakenX60(2)
    );

    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(LeftShooterIOInputs inputs) {
        sim.update(0.02);


        inputs.velocityRPM = sim.getAngularVelocityRPM();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = sim.getCurrentDrawAmps();
        inputs.motorConnected = true;
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = Math.max(-12.0, Math.min(12.0, volts));
        sim.setInputVoltage(appliedVolts);
    }

    @Override
    public void setVelocity(double velocityRPM) {
        double error = velocityRPM - sim.getAngularVelocityRPM();
        setVoltage(error * ShooterConstants.leftKP); 
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }
}