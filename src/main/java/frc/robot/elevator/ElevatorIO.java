package frc.robot.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs {
        public double positionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelsius = 0.0;
        public boolean motorConnected = true;
    }

    public void updateInputs(ElevatorIOInputs inputs);
    public void goToPosition(double meters, double expoKV);
    public void runVoltage(double volts);
    public void zeroPosition();
    public void applyIdleConfigs();
}
