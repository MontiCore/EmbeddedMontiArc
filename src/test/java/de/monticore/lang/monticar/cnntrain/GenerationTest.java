/**
 *
 *  ******************************************************************************
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
package de.monticore.lang.monticar.cnntrain;

import de.monticore.lang.monticar.cnntrain.generator.CNNTrainGenerator;
import de.se_rwth.commons.logging.Log;
import freemarker.template.TemplateException;
import org.junit.Test;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

public class GenerationTest {

    private void generate(String qualifiedName) throws IOException, TemplateException {
        Path modelPath = Paths.get("src/test/resources/valid_tests");
        CNNTrainGenerator gen =  new CNNTrainGenerator();
        gen.generate(modelPath, qualifiedName);
    }


    @Test
    public void testFullConfigGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        generate("FullConfig");
    }

    @Test
    public void testSimpleConfig1Generation() throws IOException, TemplateException {
        Log.getFindings().clear();
        generate("SimpleConfig1");
    }

    @Test
    public void testSimpleConfig2Generation() throws IOException, TemplateException {
        Log.getFindings().clear();
        generate("SimpleConfig2");
    }

    @Test
    public void testFullConfig2Generation() throws IOException, TemplateException {
        Log.getFindings().clear();
        generate("FullConfig2");
    }

}
