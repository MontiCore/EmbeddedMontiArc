/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.geometry;

import org.junit.*;

import static org.junit.Assert.*;

import de.rwth.montisim.simulation.environment.geometry.osmadapter.LinearSplineDeterminator;
import de.rwth.montisim.simulation.environment.geometry.osmadapter.SplineDeterminator;
import de.rwth.montisim.simulation.environment.osm.Parser2D;
import de.rwth.montisim.simulation.environment.osm.ParserSettings;
import de.rwth.montisim.simulation.environment.visualisationadapter.implementation.Node2D;
import de.rwth.montisim.simulation.environment.visualisationadapter.interfaces.EnvNode;
import de.rwth.montisim.simulation.environment.visualisationadapter.interfaces.EnvStreet;
import de.rwth.montisim.simulation.environment.visualisationadapter.interfaces.VisualisationEnvironmentContainer;

import java.io.InputStream;
import java.util.ArrayList;

/**
 * Created by lukas on 31.01.17.
 */
public class LinearSplineDeterminatorTest {

    @Test
    public void testApp() throws Exception {
        InputStream in = getClass().getResourceAsStream("/min_intersection_test.osm");
        Parser2D p = new Parser2D(new ParserSettings(in, ParserSettings.ZCoordinates.ALLZERO));

        p.parse();
        VisualisationEnvironmentContainer c = p.getContainer();
        ArrayList<SplineDeterminator> splines = new ArrayList<>();
        for (EnvStreet s : c.getStreets()) {
            splines.add(new LinearSplineDeterminator(s));
        }

        EnvNode n = new Node2D(0, 0, 0, 0);

        SplineDeterminator minSplineDeterminator = null;
        double minDist = Double.MAX_VALUE;
        for (SplineDeterminator s : splines) {
            //System.out.println(s.getStreet().getNodes());
            //System.out.println(s.determineSplineDistance(n));

        }
    }
}
