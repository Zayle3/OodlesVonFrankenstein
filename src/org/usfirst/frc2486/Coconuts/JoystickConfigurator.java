package org.usfirst.frc2486.Coconuts;

import org.usfirst.frc2486.Frankenstien.*;

public class JoystickConfigurator {
	
	public void function(double value){
		double a, b;
		a = 1.5;
		b = 0.5;
		
		value = (a * Math.pow(value, 3) + b * value) / (a + b);
	}
	
}
