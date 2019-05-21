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

import junit.framework.Test;
import junit.framework.TestCase;
import junit.framework.TestSuite;
import simulation.environment.object.TrafficLightSwitcher;
import simulation.environment.visualisationadapter.interfaces.SignTypeAndState;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by lukas on 13.03.17.
 */
public class TrafficSignalTest extends TestCase {
    /**
     * Create the test case
     *
     * @param testName name of the test case
     */
    public TrafficSignalTest(String testName) {
        super(testName);
    }

    /**
     * @return the suite of tests being tested
     */
    public static Test suite() {
        return new TestSuite(TrafficSignalTest.class);
    }

    public void testApp() {

    }
}