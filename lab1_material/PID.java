public class PID {
    // Current PID parameters
    private PIDParameters p;
    private double e;
    private double I;
    private double v;
    private double ad;
    private double bd;
    private double bi;
    private double D;
    private double yOld;
    private double eOld;
    private double y;

    /** Add more private variables here if needed */

    // Constructor
    public PID() {
        p = new PIDParameters();
        // Initial PID Variables
        p.N             = 11; //Derivative gain
        p.Td            = 1.5; //Derivative time
        p.Beta          = 1;
        p.H             = 0.03; //Sample time
        p.integratorOn  = true;
        p.K             = -0.14; //Gain
        p.Ti            = 7;   //Integrator time
        p.Tr            = 10;	
        this.setParameters(p);
        
        this.e 			= 0;
        this.y 			= 0;
        this.I			= 0;
        this.v			= 0;
        this.D			= 0;
        this.yOld		= 0;
        this.eOld		= 0;
        
        this.ad = p.Td/(p.Td + p.N*p.H);
        this.bd = p.K*p.N*this.ad;
        this.bi = p.K * p.H /p.Ti;

       
    }

    // Calculates the control signal v.
    // Called from BallAndBeamRegul.
    public synchronized double calculateOutput(double y, double yref) {
        /** Written by you */
    	this.e = yref - y;
    	this.y = y;
    	//Calculate P and D
    	double P = p.K*((p.Beta * yref) -y);
    	
    	//Approximation backward difference
    	this.D = this.ad*this.D + this.bd*(e-this.eOld);
    	
    	//Calculate output
    	this.v = P + this.I + this.D;
    	
    	//Saving old output 
    	return this.v;
    }

    // Updates the controller state.
    // Should use tracking-based anti-windup
    // Called from BallAndBeamRegul.
    public synchronized void updateState(double u) {
        /** Written by you */
    	if(p.integratorOn) {
    		this.I = this.I + this.bi*this.e + (u-v) * p.H/p.Tr;
    	}else {
    		this.I = 0;
    	}
    	this.yOld = this.y;
    	this.eOld = this.e;

    }

    // Returns the sampling interval expressed as a long.
    // Explicit type casting needed.
    public synchronized long getHMillis() {
        /** Written by you */
    	return (long) (p.H*1000);
    }

    // Sets the PIDParameters.
    // Called from PIDGUI.
    // Must clone newParameters.
    public synchronized void setParameters(PIDParameters newParameters) {
        /** Written by you */
    	this.p = (PIDParameters) newParameters.clone();
    	if(!p.integratorOn) this.I = 0;
    	
    	this.ad = p.Td/(p.Td + p.N*p.H);
	    this.bd = p.K*p.N*this.ad;
	    this.bi = p.K * p.H /p.Ti;
    }

    // Sets the I-part of the controller to 0.
    // For example needed when changing controller mode.	
    public synchronized void reset() {
        /** Written by you */
    	this.I = 0;
    }

    // Returns the current PIDParameters.
    public synchronized PIDParameters getParameters() {
    	return this.p;
    }
}
