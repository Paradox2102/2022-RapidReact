package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.sensors.PigeonIMU;

public class Sensor implements SensorData{
	private TalonFXSensorCollection m_leftSparkEncoder;
	private TalonFXSensorCollection m_rightSparkEncoder;
	private PigeonIMU m_gyro;
	
	public Sensor(TalonFXSensorCollection leftSparkEncoder, TalonFXSensorCollection rightSparkEncoder, PigeonIMU gyro) {
		m_leftSparkEncoder = leftSparkEncoder;
		m_rightSparkEncoder = rightSparkEncoder;
		
		m_gyro = gyro;
	}
	
	public double getLeftEncoderPos() {
		return m_leftSparkEncoder.getIntegratedSensorPosition()*Constants.k_feetPerTick;
	}
	
	public double getRightEncoderPos() {
		return m_rightSparkEncoder.getIntegratedSensorPosition()*Constants.k_feetPerTick;
	}
	
	public double getLeftEncoderVel() {
		return m_leftSparkEncoder.getIntegratedSensorVelocity()*10*Constants.k_feetPerTick;
	}
	
	public double getRightEncoderVel() {
		return m_rightSparkEncoder.getIntegratedSensorVelocity()*10*Constants.k_feetPerTick;
	}

	public PigeonIMU getGyro() {
		return m_gyro;
	}
	
	public double getAngle() {
		double[] data = new double[3];
		m_gyro.getYawPitchRoll(data);
		return data[0];
	}
}
