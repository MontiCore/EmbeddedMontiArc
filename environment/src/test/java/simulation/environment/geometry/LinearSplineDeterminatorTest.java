/**
 *
 * ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package simulation.environment.geometry;

import org.junit.*;
import static  org.junit.Assert.*;
import simulation.environment.geometry.osmadapter.LinearSplineDeterminator;
import simulation.environment.geometry.osmadapter.SplineDeterminator;
import simulation.environment.osm.Parser2D;
import simulation.environment.osm.ParserSettings;
import simulation.environment.visualisationadapter.implementation.Node2D;
import simulation.environment.visualisationadapter.interfaces.EnvNode;
import simulation.environment.visualisationadapter.interfaces.EnvStreet;
import simulation.environment.visualisationadapter.interfaces.VisualisationEnvironmentContainer;
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
        for(EnvStreet s : c.getStreets()) {
            splines.add(new LinearSplineDeterminator(s));
        }

        EnvNode n = new Node2D(0, 0, 0, 0);

        SplineDeterminator minSplineDeterminator = null;
        double minDist = Double.MAX_VALUE;
        for(SplineDeterminator s : splines) {
            //System.out.println(s.getStreet().getNodes());
            //System.out.println(s.determineSplineDistance(n));

        }
    }
}