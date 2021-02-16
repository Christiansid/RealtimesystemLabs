public class PID {
    // Current PID parameters
    private PIDParameters p;

    /** Add more private variables here if needed */

    // Constructor
    public PID() {
        p = new PIDParameters();
        // Initial PID Variables
        p.Beta          = ...;
        p.H             = ...;
        p.integratorOn  = ...;
        p.K             = ...;
        p.N             = ...;
        p.Td            = ...;
        p.Ti            = ...;
        p.Tr            = ...;
    }

    // Calculates the control signal v.
    // Called from BallAndBeamRegul.
    public synchronized double calculateOutput(double y, double yref) {
        /** Written by you */
    }

    // Updates the controller state.
    // Should use tracking-based anti-windup
    // Called from BallAndBeamRegul.
    public synchronized void updateState(double u) {
        /** Written by you */
    }

    // Returns the sampling interval expressed as a long.
    // Explicit type casting needed.
    public synchronized long getHMillis() {
        /** Written by you */
    }

    // Sets the PIDParameters.
    // Called from PIDGUI.
    // Must clone newParameters.
    public synchronized void setParameters(PIDParameters newParameters) {
        /** Written by you */
    }

    // Sets the I-part of the controller to 0.
    // For example needed when changing controller mode.	
    public synchronized void reset() {
        /** Written by you */
    }

    // Returns the current PIDParameters.
    public synchronized PIDParameters getParameters() {
        /** Written by you */
    }
}
