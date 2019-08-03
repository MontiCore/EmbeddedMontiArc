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
package simulation.environment.osm;

import org.junit.*;
import de.topobyte.osm4j.core.dataset.InMemoryMapDataSet;
import de.topobyte.osm4j.core.resolve.EntityNotFoundException;
import java.io.InputStream;
import static  org.junit.Assert.*;

/**
 * Created by lukas on 15.12.16.
 */
public class IntersectionMapperTest  {



	 @Test
    public void testApp() throws EntityNotFoundException {
        // maven test will fail for this file path

        // String filePath = "src/test/data/min_intersection_test.osm";
        InputStream in = getClass().getResourceAsStream("/min_intersection_test.osm");


        InMemoryMapDataSet data = null;


/*        OSMImporter importer = new OSMImporter(filePath);
        try {
            importer.parse();
            data = importer.getData();
        } catch (OsmInputException e) {
            e.printStackTrace();
            fail();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
            fail();
        }*/

        // Parser2D is more appropriate
        Parser2D parser2D = new Parser2D(new ParserSettings(in, ParserSettings.ZCoordinates.ALLZERO));
        try {
            parser2D.parse();
            data = parser2D.getDataSet();
        } catch (Exception e) {
            e.printStackTrace();
            fail();
        }

        IntersectionFinder finder = IntersectionFinder.getInstance();
        finder.findIntersections(data);


        IntersectionMapper mapper = new IntersectionMapper(data, finder.getIntersections());

        assertTrue(mapper.getOsmIntersectionsForWay(data.getWay(51007323)).contains(data.getNode(265316337)));
    }
}