package PID;

import lejos.hardware.port.SensorPort; //Puerto
import lejos.robotics.navigation.*; // Steer
import lejos.hardware.sensor.EV3ColorSensor; //Light Sensor
import lejos.hardware.sensor.SensorMode;


public class Run {

	public static void main(String[] args) {
		
		double Pfix = 0;
		double Ifix = 0;
		double Dfix = 0;
		
		double Kp = 6;
		double Ki = 0.0000001;
		double Kd = 40;
		
		double Derivative = 0;
		double Integral = 0;
		double Last_error = 0;
		double Error = 0;
		double sum = 0;
		int target = 40;
		
		EV3ColorSensor color = new EV3ColorSensor(SensorPort.S3);
		SensorMode colorProvider; // Proveedor de colores
		colorProvider = color.getAmbientMode(); // Se obtiene un proveedor de colores del sensor
		float[] colorArr = new float[colorProvider.sampleSize()]; // Determinar tamaño del arreglo de lectura
		
		while(true) {
			
			colorProvider = color.getAmbientMode();
			colorProvider.fetchSample(colorArr, 0);
			
			// Luz reflejada (valor normalizado entre 0 y 1 por eso la *)
			double reflectedLight = (double) colorArr[0] * 100;
			Error = reflectedLight - target;
			
			Pfix = Error * Kp;
			Integral = Integral + Error;
			Ifix = Integral * Ki;
			Derivative = Error - Last_error;
			Last_error = Error;
			Dfix = Derivative * Kd;
			sum = Pfix + Ifix + Dfix;
			
			//BAD
			//steer(sum);
			
		}
	
	}
}