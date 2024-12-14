// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotController;

public class Robot extends TimedRobot {

  private final static double kDt = 0.020;

  private final static double kv = 1.0; // Max velocity - RPS
  private final static double ka = 2.0;
  private double goal;
  private final static double kEncoderCPR = 4096;
  // private final static TrapezoidProfile.Constraints m_constraints = new
  // TrapezoidProfile.Constraints(kv, ka);

  private final XboxController controller = new XboxController(0);
  private final static ExampleSmartMotorController m_motor = new ExampleSmartMotorController(5);

  // Note: These gains are fake, and will have to be tuned for your robot.
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 1.5);

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
  private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(1.75, 0.75));

  private final TrapezoidProfile.Constraints m_Constraints = new TrapezoidProfile.Constraints(kv, ka);

  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private final  TalonSRXSimCollection m_motorSim = m_motor.getMotorSimulation();
  private final ElevSim m_elevSim = new ElevSim(m_motorSim, kEncoderCPR);
  public class ElevSim {
    private static final double kGearRatio = 40.0;
    private static final double kCarriageMassKg = 2.0;
    private static final double kDrumDiameterMeters = 2.0 / 39.37;  // Drum diameter in meters (make meter = rotation)
    private static final double kLengthMeters = 10.0;         // Maximum length in meters
    private static final double kDrumCircumMeters = kDrumDiameterMeters * Math.PI; // Drum circumference in meters

    private final ElevatorSim m_elevatorSim;
    private TalonSRXSimCollection m_motorSim;
    private double m_cpr;

    public ElevSim(TalonSRXSimCollection motorSim, double encoderCPR) {
      
        this.m_motorSim = motorSim;
        this.m_cpr = encoderCPR;

        m_elevatorSim = new ElevatorSim(DCMotor.getVex775Pro(1), kGearRatio, kCarriageMassKg, kDrumDiameterMeters / 2, -kLengthMeters, kLengthMeters, false, 0.0);
    }
    
    private TrapezoidProfile m_profile;
private final double m_kv = 1.0; // Maximum velocity in RPS
private final double m_ka = 2.0; // Maximum acceleration in RPS²
  
  public void robotInit() {
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(m_kv, m_ka);
    m_profile = new TrapezoidProfile(constraints);
    
    // Optional: Set an initial goal for the profile if needed (e.g., 0 position, no velocity)
    
    // Reset other state variables as needed
    // Note: These gains are fake, and will have to be tuned for your robot.
    m_motor.setPID(0.125, 0.0, 0.0);
    m_motor.resetEncoder();

    DataLogManager.start();

    DataLogManager.log("Initial encoder position: " + m_motor.getEncoderDistance());

  }

  
  public void teleopPeriodic() {
    m_elevSim.teleopPeriodic();
    SmartDashboard.putNumber("Goal", goal);
    SmartDashboard.putNumber("Kp", m_motor.getKp());
    SmartDashboard.putNumber("Error", m_motor.getClosedLoopError());

    SmartDashboard.putNumber("elevator rotations", m_motor.getEncoderDistance());

    double feedforward = m_feedforward.calculate(m_setpoint.velocity);  // Feedforward based on velocity
    m_motor.setSetpoint(ExampleSmartMotorController.PIDMode.kPosition, m_setpoint.position, feedforward / 12.0); // Adjust feedforward

    if (controller.getAButtonPressed()) {
      m_motor.set(0.3);
      DataLogManager.log("A button pressed");
    } else if (controller.getBButtonPressed()) {
      m_motor.set(-0.3);
      DataLogManager.log("B button pressed");
    }

    if (controller.getRawButtonPressed(8)) {
      if (m_motor.getInverted()) {
        m_motor.setInverted(false);
        DataLogManager.log("Motor Inverted = false");
      } else {
        m_motor.setInverted(true);
        DataLogManager.log("Motor Inverted = true");
      }
    }

    if (controller.getXButtonPressed()) {
      m_motor.setSetpoint(ExampleSmartMotorController.PIDMode.kPosition, m_setpoint.position, 0.0);
      goal = 4096.0;
      m_setpoint = m_profile.calculate(kDt);


    }

    if (controller.getYButtonPressed()) {
      m_motor.setSetpoint(ExampleSmartMotorController.PIDMode.kPosition, m_setpoint.position, 0.0);
      goal = 0.0;
      m_setpoint = m_profile.calculate(kDt);
    }

    if (controller.getRightBumperPressed()) {
      m_motor.stopMotor();
      DataLogManager.log("Right bumper pressed, motor stopped");
    }

    // var profile = new TrapezoidProfile(m_Constraints, m_goal, m_setpoint);
    // var profile = new TrapezoidProfile(m_Constraints, m_goal, m_setpoint);

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    // m_setpoint = m_profile.calculate(kDt);

    // Send setpoint to offboard controller PID
    // m_motor.setSetpoint(
    // ExampleSmartMotorController.PIDMode.kPosition,
    // m_setpoint.position,
    // m_feedforward.calculate(m_setpoint.velocity) / 12.0);
  }
}

  public static double getKdt() {
    return kDt;
  }
  public static double getKv() {
    return kv;
  }
  public static double getKa() {
    return ka;
  }
  public double getGoal() {
    return goal;
  }
  public void setGoal(double goal) {
    this.goal = goal;
  }
  public static double getKencodercpr() {
    return kEncoderCPR;
  }
  public XboxController getController() {
    return controller;
  }
  public static ExampleSmartMotorController getmMotor() {
    return m_motor;
  }
  public SimpleMotorFeedforward getM_feedforward() {
    return m_feedforward;
  }
  public TrapezoidProfile getM_profile() {
    return m_profile;
  }
  public TrapezoidProfile.Constraints getM_Constraints() {
    return m_Constraints;
  }
  public TrapezoidProfile.State getM_setpoint() {
    return m_setpoint;
  }
  public void setM_setpoint(TrapezoidProfile.State m_setpoint) {
    this.m_setpoint = m_setpoint;
  }
  public TalonSRXSimCollection getM_motorSim() {
    return m_motorSim;
  }
  public ElevSim getM_elevSim() {
    return m_elevSim;
  }
}