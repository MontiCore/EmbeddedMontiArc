/* (c) https://github.com/MontiCore/monticore */
package simulation.environment.osm;

import com.google.gson.Gson;
import org.junit.*;
import java.io.InputStream;
import static  org.junit.Assert.*;

/**
 * Created by Shahriar Robbani on 08-Jan-17.
 */
public class Parser2DJsonTest{

	@Test
    public void testApp() throws Exception {
        InputStream in = getClass().getResourceAsStream("/map_buildings_test.osm");
        Parser2D parser = new Parser2D(new ParserSettings(in, ParserSettings.ZCoordinates.ALLZERO));
        parser.parse();

        Gson gson = new Gson();
        String jsonString = gson.toJson(parser.getContainer());

        assertNotNull(jsonString);

        //System.out.println(jsonString);

    }
}
