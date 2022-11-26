package PID;

import lejos.hardware.port.SensorPort; //Puerto
import lejos.hardware.sensor.EV3ColorSensor; //Light Sensor
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.sensor.SensorMode;

public class Run {
	
	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;
	
	public static void main(String[] args) {
		
		double Kp = 30;
		double Ki = 0.00005;
		double Kd = 55;
		
		int target = 20;
		double Derivative = 0;
		double Integral = 0;
		double Last_error = 0;
		double Error = 0;
		double sum = 0;
		
		rightMotor = new EV3LargeRegulatedMotor(MotorPort.B);
        leftMotor = new EV3LargeRegulatedMotor(MotorPort.C);
        
		EV3ColorSensor color = new EV3ColorSensor(SensorPort.S3);
		SensorMode colorProvider; // Proveedor de colores
		colorProvider = color.getAmbientMode(); // Se obtiene un proveedor de colores del sensor
		float[] colorArr = new float[colorProvider.sampleSize()]; // Determinar tamaño del arreglo de lectura
		
		int Tp = (int)(0.15f*(leftMotor.getMaxSpeed()+rightMotor.getMaxSpeed())/2);
		
		while(true) {
			colorProvider = color.getRedMode();
			colorProvider.fetchSample(colorArr, 0);
			
			double reflectedLight = (double) colorArr[0] * 100;
			
			Error = reflectedLight - target;
			Integral = Integral + Error;
			Derivative = Error - Last_error;
			Last_error = Error;
			sum = Error*Kp + Integral*Ki + Derivative*Kd;
			
			if (sum > 0) {
				sum = sum/2000;
				sum = sum*700;
			}else if (sum < 0) {
				sum = sum/2000;
				sum = sum*700;
			}
			
			setMotorsSpeed((float)(Tp-sum), (float)(Tp+sum));
		}
	}
		
	private static void setMotorsSpeed(float _leftMotorSpeed, float _rightMotorSpeed) {
		if (_leftMotorSpeed >= 0 && _rightMotorSpeed >= 0) {

			// set speed (with the new power level)
			leftMotor.setSpeed(_leftMotorSpeed);
			rightMotor.setSpeed(_rightMotorSpeed);

			// Make the motors move forward
			leftMotor.forward();
			rightMotor.forward();
		} else if (_leftMotorSpeed < 0 && _rightMotorSpeed >= 0) {

			_leftMotorSpeed = -_leftMotorSpeed;
			
			// set speed (with the new power level)
			leftMotor.setSpeed(_leftMotorSpeed);
			rightMotor.setSpeed(_rightMotorSpeed);

				
			leftMotor.backward();
			rightMotor.forward();
		} else if (_rightMotorSpeed < 0 && _leftMotorSpeed >= 0) {

			_rightMotorSpeed = -_rightMotorSpeed;
			
			// set speed (with the new power level)
			leftMotor.setSpeed(_leftMotorSpeed);
			rightMotor.setSpeed(_rightMotorSpeed);

			leftMotor.forward();
			rightMotor.backward();
		} else {
			_leftMotorSpeed = -_leftMotorSpeed;
			_rightMotorSpeed = -_rightMotorSpeed;

			// set speed (with the new power level)
			leftMotor.setSpeed(_leftMotorSpeed);
			rightMotor.setSpeed(_rightMotorSpeed);

			// Make the motors move backward
			leftMotor.backward();
			rightMotor.backward();
		}
	}
}