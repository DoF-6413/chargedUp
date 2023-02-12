package frc.robot.SimulationDevices;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;

import edu.wpi.first.hal.SimDevice.Direction;

public class NavXWrapper extends AHRS {
  // Need to have a simulated device, plus some places to hold the values
  private SimDevice m_simDevice;
  private SimDouble m_simAngle = null;
  private SimDouble m_simRate = null;
  private SimDouble m_simPitch = null;
 
  public NavXWrapper(SPI.Port kmxp, byte update_rate_hz) {
    super(kmxp, update_rate_hz);

    // Create the SimDevice. If it returns null, we are not in simulation
    m_simDevice = SimDevice.create("AHRS", kmxp.value);
    if (m_simDevice != null) {
      m_simAngle = m_simDevice.createDouble("Angle", Direction.kOutput, 0.0);
      m_simRate = m_simDevice.createDouble("Rate", Direction.kOutput, 0.0);
      m_simPitch = m_simDevice.createDouble("Pitch", Direction.kOutput, 0.0);
    }
  }

  @Override
  public double getAngle() {
    if (m_simAngle != null) {
      return m_simAngle.get();
    }
    return super.getAngle();
  }

  @Override
  public double getRate() {
    if (m_simRate != null) {
      return m_simRate.get();
    }
    return super.getRate();
  }

  @Override
  public float getPitch() {
    if (m_simPitch != null) {
      return (float)m_simPitch.get();
    }
    return super.getPitch();
  }
}