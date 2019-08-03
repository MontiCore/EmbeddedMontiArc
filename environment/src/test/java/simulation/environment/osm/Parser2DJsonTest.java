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