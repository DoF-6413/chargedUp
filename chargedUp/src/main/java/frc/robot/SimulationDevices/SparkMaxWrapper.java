/*
 * Create a wrapper around the CANSparkMax class to support simulation
 */

package frc.robot.SimulationDevices;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj.RobotController;

/*
 * This extends the CANSparkMax class to implement simulation
 */
public class SparkMaxWrapper extends CANSparkMax {
    private SimDouble m_simSpeed;
    private SimDevice m_simSparkMax;
    private static enum direction {none};

    public SparkMaxWrapper(int deviceID, MotorType type) {
        super(deviceID,type);
        m_simSparkMax = SimDevice.create("SparkMax",deviceID);
        if (m_simSparkMax != null){
            m_simSpeed = m_simSparkMax.createDouble("speed", Direction.kOutput, 0.0);
        }
    }

    @Override
    public double get(){
        if (m_simSparkMax != null){
            return m_simSpeed.get();
        }
        return super.get();
    }

    @Override
    public void set(double speed){
        if (m_simSparkMax != null){
            m_simSpeed.set(speed);
        }else{
            super.set(speed);
        }
    }

    @Override
    public void setVoltage(double outputVolts) { //For simulation purposes, we are expecting that the battery voltage stays constant.
        if (m_simSparkMax != null){
            set(outputVolts / RobotController.getBatteryVoltage());
        } else {
            super.setVoltage(outputVolts);
        }
    }
}