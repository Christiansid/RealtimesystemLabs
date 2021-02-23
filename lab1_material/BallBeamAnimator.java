import java.util.*;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import javafx.application.Platform;
import javafx.embed.swing.JFXPanel;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.text.Text;
import javafx.beans.property.*;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial; 
import javafx.scene.shape.*;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import javafx.scene.transform.Transform;
import javafx.scene.SceneAntialiasing;
import javafx.scene.Camera;
import javafx.scene.PerspectiveCamera;
import javafx.animation.KeyFrame;
import javafx.animation.Timeline;
import javafx.util.Duration;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;

public class BallBeamAnimator {
    private static final int WIDTH = 900;
    private static final int HEIGHT = 400;
    private static final int frameduration = 40;     // animation cycle time (ms)

    private static final DoubleProperty phi = new SimpleDoubleProperty(0);  // beam angle (in radians)
    private static final DoubleProperty x = new SimpleDoubleProperty(0);    // ball position

    private static BallBeamSim ballBeamSim;                 // reference to ball and beam simulation
	private static Sphere ball;

    private ModeMonitor modeMon;
    /*
     * Constructor of class
     */
    public BallBeamAnimator(ModeMonitor modeMon) {

        //if (ballBeamSim != null) {  // Make sure ball and beam has not yet been created
        //    throw new java.lang.RuntimeException("Ball and beam simulation already exists");
        //}
        
        this.modeMon = modeMon;

        ballBeamSim = new BallBeamSim();

        SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                // This method is invoked on the EDT thread
                JFrame frame = new JFrame("Ball and Beam Animation");
                final JFXPanel fxPanel = new JFXPanel();
                frame.add(fxPanel);
                frame.setSize(WIDTH, HEIGHT);
                frame.setVisible(true);
                frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

                Platform.runLater(new Runnable() {
                    @Override
                    public void run() {
                        initFX(fxPanel);
                    }
                });
            }
        });
    }

    /*
     * Initialise GUI
     */
    private static void initFX(JFXPanel fxPanel) {          // This method is invoked on the JavaFX thread

        Box box = new Box(80, 36, 40);
        PhongMaterial material1 = new PhongMaterial();  
        material1.setDiffuseColor(Color.BLANCHEDALMOND); 
        box.setMaterial(material1);

        Cylinder cyl = new Cylinder(3,4);
        cyl.setRotationAxis(Rotate.X_AXIS);
        cyl.setRotate(90);
        cyl.setTranslateZ(-25);
        cyl.setTranslateY(2.2);

        Box beam = new Box(200, 1.5, 8);
        ball = new Sphere(4);
        ball.setTranslateY(-4.5);
        ball.setVisible(false);
        ball.translateXProperty().bind(x);

        Group ballBeam = new Group();
        ballBeam.getChildren().add(beam);
        ballBeam.getChildren().add(ball);
        ballBeam.setTranslateZ(-25);

        Group maingroup = new Group();
        maingroup.getChildren().add(box);
        maingroup.getChildren().add(cyl);
        maingroup.getChildren().add(ballBeam);

        Rotate rotateBeam = new Rotate(0, 0, 2.5, 0);	
        rotateBeam.angleProperty().bind(phi);
        ballBeam.getTransforms().add(rotateBeam);

        Scene scene = new Scene(maingroup, WIDTH, HEIGHT, false, SceneAntialiasing.BALANCED);
        scene.setFill(Color.SILVER);
        Camera camera = new PerspectiveCamera();	
        scene.setCamera(camera);

        Rotate rxBox = new Rotate(20, Rotate.X_AXIS);
        Rotate ryBox = new Rotate(20, Rotate.Y_AXIS);
        maingroup.getTransforms().addAll(rxBox, ryBox);
        maingroup.translateXProperty().set(WIDTH / 2.0);
        maingroup.translateYProperty().set(HEIGHT / 2.1);
        maingroup.translateZProperty().set(-500);

        fxPanel.setScene(scene);

        // Generate animation events
        Timeline periodicAction = new Timeline(new KeyFrame(Duration.millis(frameduration), new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {

                phi.set(0.1 * ballBeamSim.getBeamAngleWithoutNoise() * 180 / Math.PI);  // update beam angle
                x.set(-10.0 * ballBeamSim.getBallPosWithoutNoise());                    // update ball position

            }
        }));
        periodicAction.setCycleCount(Timeline.INDEFINITE);
        periodicAction.play();  // start the animation

    }

    // Retrieves beam angle from simulator
    public double getBeamAngle() { 
        return ballBeamSim.getBeamAngle();
    }

    // Retries ball position from simulator
    public double getBallPos() { 
        return ballBeamSim.getBallPos();
    }

    // Actuates u
    public void setControlSignal(double u) {
        ballBeamSim.setU(u);
    }

    // Sets the ball visibility, if ball exists
    private static void setBallVisibility(boolean visibility) {
        if (ball != null) {
            ball.setVisible(visibility);
        }
    }

    /*
     * Nested class BallBeamSim
     */
    private class BallBeamSim {

        // physical constants
        private static final double ulim = 1.0;         // control signal magnitude limit
        private static final double angOffset = 0.008;  // constant angle measurement offset
        private static final double kPhi = 4.5;         // process coefficient for the angle
        private static final double kX = 10.0;          // process coefficient for the ball position
        private static final double Tmotor = 0.05;      // motor time constant (s)
    
        // states 
        private volatile double phi = 0.0;             // beam angle
        private double xdot = 0.0;                     // ball velocity
        private volatile double x = 0.0;               // ball position
        private double tau = 0.0;                      // motor torque, tau = 1/(1+s*Tmotor)*u
        private volatile double u = 0.0;               // control signal
    
        private Random rg = new Random();
    
        public BallBeamSim() {
            final int period = 5;   // task period (in milliseconds)
    
            // Create periodic timer 
            Timer timer = new Timer();
            TimerTask timertask = new TimerTask() {
                @Override
                public void run() {
                    timeStep(period);  // time step the dynamics simulator
                }
            };
            timer.scheduleAtFixedRate(timertask,0,period);      	
        }
    
        public void timeStep(int millis) {
    
            double h = 0.001 * millis; // time step in seconds

            ModeMonitor.Mode mode = modeMon.getMode();
    
            // simulate ball movement
            switch (mode) {
                case BALL: {

                    setBallVisibility(true);

                    if (Math.abs(phi) < 0.005 && Math.abs(xdot) < 0.005) {  // stiction
                        xdot = 0.0;
                    }
                    x += (h * xdot);
                    if (x > 1.0) { x = 1.0; xdot = 0.0; }       // stop the ball from falling off the edge
                    if (x < -1.0) { x = -1.0; xdot = 0.0; }
                    xdot += (-kX * h * phi);

                    break;
                }
                case OFF:
                case BEAM: {

                    setBallVisibility(false);

                    x = 0.0;
                    xdot = 0.0;
                    break;
                }
            }
    
            // simulate beam movement
            switch (mode) {
                case BALL:
                case BEAM: {
                    phi += (kPhi * h * tau);
                    if (phi > 1.0) phi = 1.0;     // limit beam angle
                    if (phi < -1.0) phi = -1.0;
                    tau = tau + (u - tau) * h / Tmotor;
                    break;
                }
                case OFF: {
                    tau = 0.0;
                    phi = 0.0;
                    break;
                }
            }
        }
    
        public double getBeamAngleWithoutNoise() {
            return 10.0 * phi;
        }
    
        public double getBallPosWithoutNoise() {
            return 10.0 * x;
        }
    
        public double getBeamAngle() {
            return 10.0 * (phi + angOffset) + 0.0001 * rg.nextGaussian();
        }
    
        public double getBallPos() {
            return 10.0 * x + 0.01 * rg.nextGaussian() * Math.abs(xdot);
        }
    
        public void setU(double u) {
            u *= 0.1;
            if (u == 0.0)
                this.u = 0.0;
            else if (u > ulim)
                this.u = ulim;
            else if (u < -ulim)
                this.u = -ulim;
            else
                this.u = u + Math.abs(u) * 0.05 * rg.nextGaussian() + 0.0001 * rg.nextGaussian();
        }
    }
}
