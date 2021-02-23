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
    	this.opCom=opCom;
    }

    /** Sets ReferenceGenerator (called from main) */
    public void setRefGen(ReferenceGenerator refGen) {
    	this.refGen=refGen;
    }

    // Called in every sample in order to send plot data to OpCom
    private void sendDataToOpCom(double yRef, double y, double u) {
        double x = (double) (System.currentTimeMillis() - starttime) / 1000.0;
        opCom.putControlData(x, u);
        opCom.putMeasurementData(x, yRef, y);
    }

    // Sets the inner controller's parameters
    public void setInnerParameters(PIParameters p) {
    	inner.setParameters(p);
    }

    // Gets the inner controller's parameters
    public PIParameters getInnerParameters() {
    	return inner.getParameters();
    }

    // Sets the outer controller's parameters
    public void setOuterParameters(PIDParameters p) {
    	outer.setParameters(p);
    }

    // Gets the outer controller's parameters
    public PIDParameters getOuterParameters(){
    	return outer.getParameters();
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

        double yRef=0; 
		double y=0; 
		double u=0; 
		
		double PIref=0;
		double PIy=0;
		double PIu=0;
		
		double PIDref=0;
		double PIDy=0;
		double PIDu=0;
        
        while (shouldRun) {
        	PIDy = ballBeam.getBallPos();
            PIDref = refGen.getRef(); 
 
            synchronized(outer) {
 
            	PIDu = limit(outer.calculateOutput(PIDy, PIDref));
            	PIDu-=refGen.getPhiff();
            	outer.updateState(PIDu);
            	PIref=PIDu;
 
            	synchronized(inner) {
            		PIy=ballBeam.getBeamAngle();
            		PIu=limit(inner.calculateOutput(PIy, PIref));
            		PIu-=refGen.getUff();
            		inner.updateState(PIu);
            		ballBeam.setControlSignal(PIu);

            	}	
            }

            switch (modeMon.getMode()) {
                case OFF: {
                	yRef=y=u=0;
                	inner.reset();
                	outer.reset();
                	break;
                }
                case BEAM: {
                	yRef=PIref;
                	y=PIy;
                	u=PIu;
                	
                    break;
                }
                case BALL: {
                	yRef=PIDref;
                	y=PIDy;
                	u=PIDu;

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
