/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.visualization;

import org.graphstream.graph.Graph;
import org.graphstream.ui.view.Viewer;
import org.graphstream.ui.view.ViewerListener;
import org.graphstream.ui.view.ViewerPipe;

import javax.swing.*;

public class SimpleModelViewer implements ViewerListener {

    private Graph graph;
    protected boolean loop = true;

    public SimpleModelViewer(Graph g) {
        this.graph = g;
    }

    public void run() {
        Viewer viewer = this.graph.display();
        viewer.getDefaultView().add(new JLabel(graph.getId().toString()));
        viewer.setCloseFramePolicy(Viewer.CloseFramePolicy.CLOSE_VIEWER);   // set to "HIDE_ONLY" to allow for further communication
        ViewerPipe fromViewer = viewer.newViewerPipe();
        fromViewer.addViewerListener(this);
        //fromViewer.addSink(this.graph);
        while(loop) {
            fromViewer.pump();
        }
    }

    public void viewClosed(String id) {
        this.loop = false;
    }

    public void buttonPushed(String id) {
        //System.out.println("Button pushed on node "+id);
    }

    public void buttonReleased(String id) {
        //System.out.println("Button released on node "+id);
    }

}
