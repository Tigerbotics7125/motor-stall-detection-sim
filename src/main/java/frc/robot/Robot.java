// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  private TalonFX m_talon;

  private DCMotorSim m_motorSim;

  private double m_currentDraw = 0;

  private Debouncer m_debouncer;
  private double m_stallCurrent;

  @Override
  public void robotInit() {
    DriverStation.silenceJoystickConnectionWarning(true);

    m_talon = new TalonFX(1);

    TalonFXConfiguration config = new TalonFXConfiguration();

    // config.CurrentLimits.StatorCurrentLimit = 80;
    // config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Slot0.kP = 1;

    m_talon.getConfigurator().apply(config);

    var motor = DCMotor.getNeoVortex(1);
    m_motorSim = new DCMotorSim(motor, 20, .25);

    // time in seconds value must remain true to be considered true by the
    // debouncer.
    double debounceTime = .3;
    // debouncer is looking for true values.
    DebounceType debounceType = DebounceType.kRising;
    m_debouncer = new Debouncer(debounceTime, debounceType);

    SmartDashboard.putNumber("Stall Current", 35);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Talon Torque Current", m_talon.getTorqueCurrent().getValue());
    SmartDashboard.putNumber("Talon Supply Current", m_talon.getSupplyCurrent().getValue());
    SmartDashboard.putNumber("Applied Voltage", m_talon.getMotorVoltage().getValue());
    SmartDashboard.putNumber("Applied Duty Cycle", m_talon.getDutyCycle().getValue());
    SmartDashboard.putNumber("Motor Velocity RPS", m_talon.getVelocity().getValue());
    m_stallCurrent = SmartDashboard.getNumber("Stall Current", 35);
  }

  @Override
  public void simulationInit() {
  }

  boolean shouldStallMotor = true;

  @Override
  public void simulationPeriodic() {
    TalonFXSimState talonSim = m_talon.getSimState();

    // simulate the motor
    m_motorSim.setInputVoltage(m_talon.getMotorVoltage().getValue());
    m_motorSim.update(.02);

    // update the talon
    var time = Timer.getFPGATimestamp() % 10;

    if (time <= .02)
      shouldStallMotor = true;
    if (time >= 5)
      shouldStallMotor = false;
    if (SmartDashboard.getBoolean("Motor Stall Detected", true))
      shouldStallMotor = false;

    if (shouldStallMotor || time < 5) {
      // simulate a stall condition.
      talonSim.setRotorVelocity(.9 * m_talon.getRotorVelocity().getValue());
      // keep the motor sim updated.
      m_motorSim.setState(Units.rotationsToRadians(m_talon.getRotorPosition().getValue()), Units.rotationsToRadians(m_talon.getVelocity().getValue()));
    } else {
      talonSim.setRawRotorPosition(m_motorSim.getAngularPositionRotations());
      talonSim.setRotorVelocity(Units.radiansToRotations(m_motorSim.getAngularVelocityRadPerSec()));
    }

    // set the battery voltage
    m_currentDraw = m_talon.getSupplyCurrent().getValue();
    // m_currentDraw = talonSim.getSupplyCurrent();
    double batteryVoltage = BatterySim.calculateLoadedBatteryVoltage(13.6, .02, m_currentDraw);
    RoboRioSim.setVInVoltage(batteryVoltage);
    talonSim.setSupplyVoltage(batteryVoltage);

  }

  @Override
  public void disabledInit() {
    m_currentDraw = 0;
    // Disable the motor.
    m_talon.disable();
    m_motorSim.setInputVoltage(0);
  }

  @Override
  public void teleopInit() {
  }

  boolean hasStalled = false;

  @Override
  public void teleopPeriodic() {

    boolean isStalled = m_debouncer.calculate(m_currentDraw > m_stallCurrent);
    SmartDashboard.putBoolean("Motor Stall Detected", isStalled);

    if (isStalled)
      hasStalled = true;
    if (Timer.getFPGATimestamp() % 10 > 5)
      hasStalled = false;

    if (isStalled || hasStalled) {
      m_talon.setControl(new NeutralOut());
    } else {
      m_talon.setControl(new VelocityVoltage(6));
    }

    PowerDistribution pdp = new PowerDistribution();
    pdp.getCurrent(1);
  }
}
