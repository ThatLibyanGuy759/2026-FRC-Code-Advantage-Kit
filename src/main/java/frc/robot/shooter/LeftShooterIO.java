package frc.robot.shooter;
 
import org.littletonrobotics.junction.AutoLog;
 

public interface LeftShooterIO {
 
 
    @AutoLog
    public static class LeftShooterIOInputs {
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public boolean motorConnected = true;
    }
 
    public default void updateInputs(LeftShooterIOInputs inputs) {

    }
 

    public default void setVoltage(double volts) {
        
    }
 

    public default void setVelocity(double velocityRPM) {
        
    }

    public default void stop() {}
}


