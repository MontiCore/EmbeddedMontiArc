/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.visualization;

import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.filesystem.MapData;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.Pair;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.util.Map;

public class MapModel {
    MapData data;
    
    public MapModel(MapData data){
        this.data = data;
    }

    public void paintMap(Graphics g, KindaViewMatrix view){
        Graphics2D g2 = (Graphics2D)g;

        Color default_color = new Color(150,150,150);
        g.setColor(default_color);
        render(g, view, false);

        Color driveable_color = new Color(130,130,130);
        g.setColor(driveable_color);
        g2.setStroke(new BasicStroke(2));
        render(g, view, true);
    }


    private void render(Graphics g, KindaViewMatrix view, boolean driveable){
        for (Map.Entry<Long, MapData.Way> c : data.ways.entrySet()){
            MapData.Way way = c.getValue();
            if (driveable && !way.driveable) continue;
            else if (!driveable && way.driveable) continue;
            int size = way.nodes.size();
            if (size < 2) continue;
            int x[] = new int[size];
            int y[] = new int[size];
            int i = 0;
            for (Long nid : way.nodes){
                MapData.Node node = data.nodes.get(nid);
                if (node != null){
                    Pair<Integer, Integer> res = view.get_screen_coords(node.point);
                    x[i] = res.getKey();
                    y[i] = res.getValue();
                    i++;
                }
            }
            if (i>0){
                g.drawPolyline(x, y, i);
            }
        }
    }
}