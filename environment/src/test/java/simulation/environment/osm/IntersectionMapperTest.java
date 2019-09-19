/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
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
