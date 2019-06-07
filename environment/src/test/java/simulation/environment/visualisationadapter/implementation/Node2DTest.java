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
package simulation.environment.visualisationadapter.implementation;

import static org.junit.Assert.*;
import org.junit.*;
import simulation.environment.visualisationadapter.interfaces.EnvIntersection;
import simulation.environment.visualisationadapter.interfaces.EnvNode;
import java.util.ArrayList;

/**
 * Created by lukas on 22.01.17.
 */
public class Node2DTest  {

	@Test
    public void testApp() throws Exception {
        EnvNode n1 = new Node2D(2,1,10);
        EnvNode n2 = new Node2D(2,1,10);
        EnvNode n3 = new Node2D(223,2,11);
        EnvNode n4 = new Node2D(26,14,12);
        EnvNode n5 = new Node2D(25,11,13);
        EnvIntersection n6 = new Intersection2D(2,1,10);

        ArrayList<EnvNode> nodes = new ArrayList<>();
        nodes.add(n1);
        nodes.add(n3);
        nodes.add(n4);
        nodes.add(n5);
        nodes.add(n1);

        assertTrue(n1.equals(n2));
        assertTrue(n2.equals(n1));
        assertFalse(n1.equals(n3));

        assertTrue(n6.equals(n1));
        assertTrue(n1.equals(n6));

        assertTrue(nodes.contains(n2));
        assertTrue(nodes.contains(new Node2D(2,1,10)));

        assertEquals(0, nodes.indexOf(n1));
        assertEquals(nodes.indexOf(n1), nodes.indexOf(n6));

        assertEquals(4, nodes.lastIndexOf(new Node2D(2,1,10)));

    }
}