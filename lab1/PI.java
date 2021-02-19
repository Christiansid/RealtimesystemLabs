public class PI {
    // Current PI parameters
    private PIParameters p;
    private double e;
    private double v;
    private double I;
    private final double bi;

    /** Add more private variables here if needed */

    // Constructor
    public PI() {
        p = new PIParameters();
        // Initial PI Variables
        p.Beta          = 1;
        p.H             = 1;
        p.integratorOn  = true;
        p.K             = 1;
        p.Ti            = 1;
        p.Tr            = 1;
        this.e 			= 0;
        this.I 			= 0;
        
        this.bi = p.K * p.H /p.Ti;
    }

    // Calculates the control signal v.
    // Called from BeamRegul.
    public synchronized double calculateOutput(double y, double yref) {
        /** Written by you */
    	//Calculate error
    	this.e = yref - y;
    	//Calculate P 
    	double P = p.K*(p.Beta * yref-y);
    	//Calculate and return output
    	this.v = P+this.I;
    	// this.v = this.v < -(p.Tr) ? -(p.Tr) : (this.v > p.Tr ? p.Tr: this.v);
    	return this.v;
    }

    // Updates the controller state.
    // Should use tracking-based anti-windup
    // Called from BeamRegul.
    public synchronized void updateState(double u) {
        /** Written by you */
    	if(p.integratorOn) {
    		this.I = this.I + this.bi*this.e + (u-v) * p.H/p.Tr;
    	}else {
    		this.I = 0;
    	}
    }

    // Returns the sampling interval expressed as a long.
    // Note: Explicit type casting needed
    public synchronized long getHMillis() {
        /** Written by you */
    	return (long) (p.H*1000);
    }

    // Sets the PIParameters.
    // Called from PIGUI.
    // Must clone newParameters.
    public synchronized void setParameters(PIParameters newParameters) {
        /** Written by you */
    	this.p = (PIParameters) newParameters.clone();
    	if(p.integratorOn) this.I = 0;
    }

    // Sets the I-part of the controller to 0.
    // For example needed when changing controller mode.
    public synchronized void reset() {
        /** Written by you */
    	this.I = 0;
    }

    // Returns the current PIParameters.
    public synchronized PIParameters getParameters() {
        /** Written by you */
    	return this.p;
    }
}
