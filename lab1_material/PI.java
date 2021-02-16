public class PI {
    // Current PI parameters
    private PIParameters p;

    /** Add more private variables here if needed */

    // Constructor
    public PI() {
        p = new PIParameters();
        // Initial PI Variables
        p.Beta          = ...;
        p.H             = ...;
        p.integratorOn  = ...;
        p.K             = ...;
        p.Ti            = ...;
        p.Tr            = ...;
    }

    // Calculates the control signal v.
    // Called from BeamRegul.
    public synchronized double calculateOutput(double y, double yref) {
        /** Written by you */
    }

    // Updates the controller state.
    // Should use tracking-based anti-windup
    // Called from BeamRegul.
    public synchronized void updateState(double u) {
        /** Written by you */
    }

    // Returns the sampling interval expressed as a long.
    // Note: Explicit type casting needed
    public synchronized long getHMillis() {
        /** Written by you */
    }

    // Sets the PIParameters.
    // Called from PIGUI.
    // Must clone newParameters.
    public synchronized void setParameters(PIParameters newParameters) {
        /** Written by you */
    }

    // Sets the I-part of the controller to 0.
    // For example needed when changing controller mode.
    public synchronized void reset() {
        /** Written by you */
    }

    // Returns the current PIParameters.
    public synchronized PIParameters getParameters() {
        /** Written by you */
    }
}
