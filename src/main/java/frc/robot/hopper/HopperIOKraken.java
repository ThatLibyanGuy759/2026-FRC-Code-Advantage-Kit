package frc.robot.hopper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class HopperIOKraken implements HopperIO {

    private final TalonFX hopperMotor;
    private final HopperConfigs configs;
    private final VoltageOut voltageOut = new VoltageOut(0);
    private final MotionMagicExpoVoltage setpointRequest = new MotionMagicExpoVoltage(0);

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentSignal;
    private final StatusSignal<Temperature> tempSignal;

    public HopperIOKraken() {
        configs = new HopperConfigs();
        hopperMotor = new TalonFX(HopperConstants.hopperMotorID);

        for (int i = 0; i < 5; i++) {
            var status = hopperMotor.getConfigurator().apply(configs.hopperMotorConfig());
            if (status.isOK()) break;
        }
        positionSignal = hopperMotor.getPosition();
        velocitySignal = hopperMotor.getVelocity();
        voltageSignal = hopperMotor.getMotorVoltage();
        currentSignal = hopperMotor.getSupplyCurrent();
        tempSignal = hopperMotor.getDeviceTemp();


        BaseStatusSignal.setUpdateFrequencyForAll(50.0, positionSignal, velocitySignal, voltageSignal, currentSignal, tempSignal);

        hopperMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.motorConnected = BaseStatusSignal.refreshAll(positionSignal, velocitySignal, voltageSignal, currentSignal, tempSignal).isOK();

        inputs.positionMeters = positionSignal.getValueAsDouble() * HopperConstants.metersPerRotation;
        inputs.velocityMetersPerSec = velocitySignal.getValueAsDouble() * HopperConstants.metersPerRotation;
        inputs.appliedVolts = voltageSignal.getValueAsDouble();
        inputs.currentAmps = currentSignal.getValueAsDouble();
        inputs.tempCelsius = tempSignal.getValueAsDouble();
    }

    @Override
    public void goToPosition(double meters, double expoKV) {
        MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
        mmConfigs.MotionMagicExpo_kV = expoKV;
        mmConfigs.MotionMagicExpo_kA = HopperConstants.expoKA;
        hopperMotor.getConfigurator().apply(mmConfigs);
        double targetRotations = meters / HopperConstants.metersPerRotation;
        hopperMotor.setControl(setpointRequest.withPosition(targetRotations));
    }

    @Override
    public void runVoltage(double volts) {
        hopperMotor.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void zeroPosition() {
        hopperMotor.setPosition(0.0);
    }

    @Override
    public void applyIdleConfigs() {
        hopperMotor.getConfigurator().apply(configs.idleHopperMotorConfig());
    }
}