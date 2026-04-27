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
 
    public default void updateInputs(HopperIOInputs inputs) {}
    public default void goToPosition(double meters, double expoKV) {}
    public default void runVoltage(double volts) {}
    public default void zeroPosition() {}
    public default void applyIdleConfigs() {}
}
 