package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevSim
{
    private static final double   kGearRatio          = 40.0;
    private static final double   kCarriageMassKg     = 2.0;
    private static final double   kDrumDiameterMeters = 2.0 / 39.37;  // Drum diameter in meters (make meter = rotation)
    private static final double   kLengthMeters       = 10.0;         // Maximum length in meters
    private static final double   kDrumCircumMeters   = kDrumDiameterMeters * Math.PI;      // Drum diameter in meters
    // private static final double   kRolloutRatioMeters = kDrumCircumMeters / kGearRatio;     // Meters per shaft rotation

    private final ElevatorSim     m_elevatorSim       = new ElevatorSim(DCMotor.getVex775Pro(1), kGearRatio, kCarriageMassKg,
            kDrumDiameterMeters / 2, -kLengthMeters, kLengthMeters, false, 0.0);

    private TalonSRXSimCollection m_motorSim;
    private double                m_cpr;

    public ElevSim(TalonSRXSimCollection motorSim, double encoderCPR)
    {
        TalonSRXSimCollection motorSim2 = motorSim;
        m_motorSim = motorSim2;
        m_cpr = encoderCPR;

    }

    public void periodic( )
    {
        m_motorSim.setBusVoltage(RobotController.getInputVoltage( ));
        m_elevatorSim.setInput(m_motorSim.getMotorOutputLeadVoltage( ));

        m_elevatorSim.update(0.020);

        m_motorSim.setQuadratureRawPosition((int) (m_cpr * m_elevatorSim.getPositionMeters( ) / kDrumCircumMeters));
        m_motorSim.setQuadratureVelocity((int) (m_cpr * (m_elevatorSim.getVelocityMetersPerSecond( ) / kDrumCircumMeters) / 10));

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps( )));

        SmartDashboard.putNumber("SIM-motorVolts", m_motorSim.getMotorOutputLeadVoltage( ));
        SmartDashboard.putNumber("SIM-elevPos", m_elevatorSim.getPositionMeters( ));
        SmartDashboard.putNumber("SIM-elevVel", m_elevatorSim.getVelocityMetersPerSecond( ));

    }

    public void reset( )
    {
        m_motorSim.setQuadratureRawPosition((int) (m_cpr * m_elevatorSim.getPositionMeters( ) / kDrumCircumMeters));
        m_motorSim.setQuadratureVelocity((int) (m_cpr * (m_elevatorSim.getVelocityMetersPerSecond( ) / kDrumCircumMeters) / 10));

        m_elevatorSim.setState(0.0, 0.0);
        m_elevatorSim.setInput(0.0);
    }
}
