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
    	this.opCom = opCom;
    }

    /** Sets ReferenceGenerator (called from main) */
    public void setRefGen(ReferenceGenerator refGen) {
        /** Written by you */
    	this.refGen = refGen;
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
    	this.inner.setParameters(p);
    }

    // Gets the inner controller's parameters
    public PIParameters getInnerParameters() {
        /** Written by you */
    	return this.inner.getParameters();
    }

    // Sets the outer controller's parameters
    public void setOuterParameters(PIDParameters p) {
        /** Written by you */
    	this.outer.setParameters(p);
    }

    // Gets the outer controller's parameters
    public PIDParameters getOuterParameters(){
        /** Written by you */
    	return this.outer.getParameters();
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
        	double yRef = this.refGen.getRef();
        	double y;
        	double u;
            switch (modeMon.getMode()) {
                case OFF: {
                    /** Written by you */
                	shouldRun = false;
                	this.inner.reset();
                	this.outer.reset();
                	y = 0;
                	u = 0;
                    break;
                }
                case BEAM: {
                    /** Written by you */
            		y = this.ballBeam.getBeamAngle();
                	synchronized(this.inner) {
                		double v = this.inner.calculateOutput(y, yRef);
                		u = this.limit(v);
                		this.inner.updateState(u);
                	}

                    break;
                }
                case BALL: {
                    /** Written by you */
            		y = this.ballBeam.getBallPos();
                	synchronized(this.outer) {
                		double v = this.outer.calculateOutput(y, yRef);
                		u = this.limit(v);
                		this.outer.updateState(u);
                	}
                	
                    break;
                }
                default: {
                    System.out.println("Error: Illegal mode.");
                    y = 0;
                    u = 0;
                    break;
                }
            }
    		this.ballBeam.setControlSignal(u);
            this.sendDataToOpCom(yRef, y, u);

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
