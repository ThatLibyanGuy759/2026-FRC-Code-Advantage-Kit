package frc.robot.hopper;
 
import org.littletonrobotics.junction.AutoLog;
 
public interface HopperIO {
 
    @AutoLog
    public static class HopperIOInputs {
        public double positionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelsius = 0.0;
        public boolean motorConnected = true;
    }
 
    public void updateInputs(HopperIOInputs inputs);
    public void goToPosition(double meters, double expoKV);
    public void runVoltage(double volts);
    public void zeroPosition();
    public void applyIdleConfigs();
}
 