public class Regul extends Thread {

    private PI inner = new PI();
    private PID outer = new PID();

    private BallBeamAnimator ballBeam;

    private ReferenceGenerator refGen;
    private OpCom opCom;

    private int priority;
    private boolean shouldRun = true;
    private long starttime;

    private ModeMonitor modeMon;

    public Regul(int pri, ModeMonitor modeMon) {
        priority = pri;
        setPriority(priority);

        ballBeam = new BallBeamAnimator(modeMon);

        this.modeMon = modeMon;
    }

    /** Sets OpCom (called from main) */
    public void setOpCom(OpCom opCom) {
        /** Written by you */
    }

    /** Sets ReferenceGenerator (called from main) */
    public void setRefGen(ReferenceGenerator refGen) {
        /** Written by you */
    }

    // Called in every sample in order to send plot data to OpCom
    private void sendDataToOpCom(double yRef, double y, double u) {
        double x = (double) (System.currentTimeMillis() - starttime) / 1000.0;
        opCom.putControlData(x, u);
        opCom.putMeasurementData(x, yRef, y);
    }

    // Sets the inner controller's parameters
    public void setInnerParameters(PIParameters p) {
        /** Written by you */
    }

    // Gets the inner controller's parameters
    public PIParameters getInnerParameters() {
        /** Written by you */
    }

    // Sets the outer controller's parameters
    public void setOuterParameters(PIDParameters p) {
        /** Written by you */
    }

    // Gets the outer controller's parameters
    public PIDParameters getOuterParameters(){
        /** Written by you */
    }

    // Called from OpCom when shutting down
    public void shutDown() {
        shouldRun = false;
    }

    // Saturation function
    private double limit(double v) {
        return limit(v, -10, 10);
    }

    // Saturation function
    private double limit(double v, double min, double max) {
        if (v < min) v = min;
        else if (v > max) v = max;
        return v;
    }

    public void run() {

        long duration;
        long t = System.currentTimeMillis();
        starttime = t;

        while (shouldRun) {
            /** Written by you */

            switch (modeMon.getMode()) {
                case OFF: {
                    /** Written by you */
                    break;
                }
                case BEAM: {
                    /** Written by you */
                    break;
                }
                case BALL: {
                    /** Written by you */
                    break;
                }
                default: {
                    System.out.println("Error: Illegal mode.");
                    break;
                }
            }

            sendDataToOpCom(yRef, y, u);

            // sleep
            t = t + inner.getHMillis();
            duration = t - System.currentTimeMillis();
            if (duration > 0) {
                try {
                    sleep(duration);
                } catch (InterruptedException x) {}
            } else {
                System.out.println("Lagging behind...");
            }
        }
        ballBeam.setControlSignal(0.0);
    }
}
