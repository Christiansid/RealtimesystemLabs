public class PID {
    // Current PID parameters
    private PIDParameters p;

    private double I = 0; // Integral part of PID
    private double D = 0; // Derivative part of PID
    private double v = 0; // Computed control signal
    private double e = 0; // Error signal
    private double y = 0; // Measurement signal
    private double yOld = 0; // Old measurement signal
    private double ad; // Help variable for Derivative calculation
    private double bd; // Help variable for Derivative calculation

    // Constructor
    public PID() {
        p = new PIDParameters();
        // Initial PID Variables
        p.Beta = 1.0;
		p.H = 0.1;
		p.integratorOn = false;
		p.K = -0.1;
		p.Ti = 0.0;
		p.Tr = 10.0;
		p.Td = 4.0; 
		p.N = 7.0; 
		setParameters(p); 
    }

    // Calculates the control signal v.
    // Called from BallAndBeamRegul.
    public synchronized double calculateOutput(double y, double yref) {
    	this.y = y; 
		e = yref-y;
		ad = p.Td/(p.Td+p.N*p.H); 
		bd = p.K*p.N*ad; 
		D = D*ad-bd*(y-yOld);
		v = p.K*((p.Beta*yref)-y) + I + D;
		return v;
    }

    // Updates the controller state.
    // Should use tracking-based anti-windup
    // Called from BallAndBeamRegul.
    public synchronized void updateState(double u) {
    	if(p.integratorOn) {
			double ar=p.H/p.Tr;
			double bi=p.H/p.Ti;
			I+=(p.K*bi*e)+(ar*(u-v));
 
		}else {
			I=0.0;
		}	
		yOld = y; 
    }

    // Returns the sampling interval expressed as a long.
    // Explicit type casting needed.
    public synchronized long getHMillis() {
    	return (long) (p.H*1000);
    }

    // Sets the PIDParameters.
    // Called from PIDGUI.
    // Must clone newParameters.
    public synchronized void setParameters(PIDParameters newParameters) {
    	p=(PIDParameters)newParameters.clone();
		if(!p.integratorOn) {
			I=0.0;
		}
    }

    // Sets the I-part of the controller to 0.
    // For example needed when changing controller mode.	
    public synchronized void reset() {
        I=0.0;
    }

    // Returns the current PIDParameters.
    public synchronized PIDParameters getParameters() {
    	return p;
    }
}
