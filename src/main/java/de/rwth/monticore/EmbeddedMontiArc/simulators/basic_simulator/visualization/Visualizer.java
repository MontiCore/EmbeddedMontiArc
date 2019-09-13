/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore
 */

package de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.visualization;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Map;

import javax.swing.JPanel;
import javax.swing.Timer;

import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.controller.SimulationResult;
import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.controller.SimulationResult.CarFrame;
import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.controller.SimulationResult.CarInfo;
import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.controller.SimulationResult.SimulationFrame;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.Point2D;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.Pair;

public class Visualizer extends JPanel implements ActionListener {
    long time;
    Timer timer = new Timer(16, this);
    public boolean antialiasing;

    public SimulationResult simulation_result;
    public MapModel map_model;
    

    public Visualizer(boolean auto_refresh) {
        antialiasing = false;
        simulation_result = null;
        map_model = null;
        if (auto_refresh) timer.start();
        time = System.currentTimeMillis();
    }

    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D)g;
        if (antialiasing){
            RenderingHints rh = new RenderingHints(
                    RenderingHints.KEY_TEXT_ANTIALIASING,
                    RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
            rh.put(RenderingHints.KEY_ANTIALIASING,
                    RenderingHints.VALUE_ANTIALIAS_ON);
            g2.setRenderingHints(rh);
        }
        Dimension d = this.getSize();
        int area_width = (int) d.getWidth()-1;
        int area_height = (int) d.getHeight()-1;

        if (map_model != null){
            KindaViewMatrix view = new KindaViewMatrix(d, map_model.data.size, map_model.data.min_corner);

            map_model.paintMap(g, view);


            if (simulation_result != null){
                g.setColor(new Color(0,191,255));
                /* 
                g.fillRect(5, 5, 5, 5); */

                g2.setStroke(new BasicStroke(3));
    
                for (Map.Entry<Long, CarInfo> c : simulation_result.cars.entrySet()){
                    CarInfo info = c.getValue();
                    int x[] = new int[info.planned_trajectory.length];
                    int y[] = new int[info.planned_trajectory.length];
                    int i = 0;
                    for (Point2D p : info.planned_trajectory){
                        Pair<Integer, Integer> res = view.get_screen_coords(p);
                        x[i] = res.getKey();
                        y[i] = res.getValue();
                        i++;
                    }
                    g2.drawPolyline(x, y, x.length);
                }
    
                g.setColor(new Color(255,69,0));
                double tick_time = 1000.0 / simulation_result.simulation_frequency;
                double playback_speed = 4.0;
                long temp_vis_frame = ((long) ((time*playback_speed) / tick_time) % simulation_result.frames.size());
                SimulationFrame temp_frame = simulation_result.frames.get((int) temp_vis_frame);
                for (Map.Entry<Long, CarFrame> c : temp_frame.car_frames.entrySet()){
                    CarFrame cf = c.getValue();
                    Pair<Integer, Integer> res = view.get_screen_coords(new Point2D(cf.x, cf.y));
                    g.fillOval(
                        res.getKey()-4, res.getValue()-4, 
                        8, 8);
                }
            }
        }

        
        /* g.setColor(Color.CYAN);
        g.drawRect(0, 0, area_width, area_height);
        g.setColor(Color.RED);
        double t = time*0.001*2;
        g.fillOval(100 + (int) (Math.sin(t)*30),100 + (int) (Math.cos(t)*30),10,10); */
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        if (e.getSource() == timer) {
            time = System.currentTimeMillis();
            repaint();
            revalidate();
        }
    }

}

/*
 * package de.rwth_aachen.se.montisim.simulators.basic_simulator.visualization;
 * 
 * import java.awt.BorderLayout;
 * 
 * import javax.swing.JPanel;
 * 
 * import com.jogamp.opengl.awt.GLCanvas; import com.jogamp.opengl.glu.GLU;
 * import com.jogamp.opengl.GL; import com.jogamp.opengl.GL2; import
 * com.jogamp.opengl.GLAnimatorControl; import com.jogamp.opengl.GLAutoDrawable;
 * import com.jogamp.opengl.GLEventListener; import com.jogamp.opengl.GLProfile;
 * 
 * public class Visualizer extends GLCanvas implements GLEventListener {
 * 
 * protected static void setup( GL2 gl2, int width, int height ) {
 * gl2.glMatrixMode( GL2.GL_PROJECTION ); gl2.glLoadIdentity();
 * 
 * // coordinate system origin at lower left with width and height same as the
 * window GLU glu = new GLU(); glu.gluOrtho2D( 0.0f, width, 0.0f, height );
 * 
 * gl2.glMatrixMode( GL2.GL_MODELVIEW ); gl2.glLoadIdentity();
 * 
 * gl2.glViewport( 0, 0, width, height ); }
 * 
 * protected static void render( GL2 gl2, int width, int height ) { gl2.glClear(
 * GL.GL_COLOR_BUFFER_BIT );
 * 
 * // draw a triangle filling the window gl2.glLoadIdentity(); gl2.glBegin(
 * GL.GL_TRIANGLES ); gl2.glColor3f( 1, 0, 0 ); gl2.glVertex2f( 0, 0 );
 * gl2.glColor3f( 0, 1, 0 ); gl2.glVertex2f( width, 0 ); gl2.glColor3f( 0, 0, 1
 * ); gl2.glVertex2f( width / 2, height ); gl2.glEnd(); }
 * 
 * @Override public void reshape( GLAutoDrawable glautodrawable, int x, int y,
 * int width, int height ) { setup( glautodrawable.getGL().getGL2(), width,
 * height ); }
 * 
 * @Override public void init( GLAutoDrawable glautodrawable ) { }
 * 
 * @Override public void dispose( GLAutoDrawable glautodrawable ) { }
 * 
 * @Override public void display( GLAutoDrawable glautodrawable ) { render(
 * glautodrawable.getGL().getGL2(), glautodrawable.getSurfaceWidth(),
 * glautodrawable.getSurfaceHeight() ); }
 * 
 * private GLAnimatorControl glanimatorcontrol;
 * 
 * public void init() { GLProfile.initSingleton(); addGLEventListener( this );
 * 
 * setSize( getSize() ); //glanimatorcontrol = new FPSAnimator( glcanvas, 30 );
 * } }
 */