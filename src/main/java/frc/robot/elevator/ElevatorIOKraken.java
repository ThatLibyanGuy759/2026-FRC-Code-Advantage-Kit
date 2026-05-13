package frc.robot.elevator;

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

public class ElevatorIOKraken implements ElevatorIO {

    private final TalonFX elevatorMotor;
    private final ElevatorConfigs configs;
    private final VoltageOut voltageOut = new VoltageOut(0);
    private final MotionMagicExpoVoltage setpointRequest = new MotionMagicExpoVoltage(0);

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentSignal;
    private final StatusSignal<Temperature> tempSignal;

    public ElevatorIOKraken() {
        configs = new ElevatorConfigs();
        elevatorMotor = new TalonFX(ElevatorConstants.elevatorMotorID);

        for (int i = 0; i < 5; i++) {
            var status = elevatorMotor.getConfigurator().apply(configs.elevatorMotorConfig());
            if (status.isOK()) break;
        }
        positionSignal = elevatorMotor.getPosition();
        velocitySignal = elevatorMotor.getVelocity();
        voltageSignal = elevatorMotor.getMotorVoltage();
        currentSignal = elevatorMotor.getSupplyCurrent();
        tempSignal = elevatorMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, positionSignal, velocitySignal, voltageSignal, currentSignal, tempSignal);

        elevatorMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.motorConnected = BaseStatusSignal.refreshAll(positionSignal, velocitySignal, voltageSignal, currentSignal, tempSignal).isOK();

        inputs.positionMeters = positionSignal.getValueAsDouble() * ElevatorConstants.metersPerRotation;
        inputs.velocityMetersPerSec = velocitySignal.getValueAsDouble() * ElevatorConstants.metersPerRotation;
        inputs.appliedVolts = voltageSignal.getValueAsDouble();
        inputs.currentAmps = currentSignal.getValueAsDouble();
        inputs.tempCelsius = tempSignal.getValueAsDouble();
    }

    @Override
    public void goToPosition(double meters, double expoKV) {
        MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
        mmConfigs.MotionMagicExpo_kV = expoKV;
        mmConfigs.MotionMagicExpo_kA = ElevatorConstants.expoKA;
        elevatorMotor.getConfigurator().apply(mmConfigs);
        double targetRotations = meters / ElevatorConstants.metersPerRotation;
        elevatorMotor.setControl(setpointRequest.withPosition(targetRotations));
    }

    @Override
    public void runVoltage(double volts) {
        elevatorMotor.getConfigurator().apply(configs.idleelevatorMotorConfig());
        elevatorMotor.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void zeroPosition() {
        elevatorMotor.setPosition(0.0);
    }

    @Override
    public void applyIdleConfigs() {
        elevatorMotor.getConfigurator().apply(configs.idleelevatorMotorConfig());
    }
}
