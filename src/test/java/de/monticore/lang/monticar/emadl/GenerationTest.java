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
package de.monticore.lang.monticar.emadl;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.emadl.generator.AbstractSymtab;
import de.monticore.lang.monticar.emadl.generator.EMADLGenerator;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.Splitters;
import de.se_rwth.commons.logging.Log;
import freemarker.template.TemplateException;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Scanner;

import static de.monticore.lang.monticar.emadl.ParserTest.ENABLE_FAIL_QUICK;
import static junit.framework.TestCase.assertEquals;
import static junit.framework.TestCase.assertTrue;

public class GenerationTest {

    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(ENABLE_FAIL_QUICK);
    }

    private void generate(String qualifiedName) throws IOException, TemplateException{
        EMADLGenerator gen =  new EMADLGenerator();
        gen.generate("src/test/resources/models/", qualifiedName);
    }

    private List<FileContent> generateStrings(String modelsDirPath, String qualifiedName) throws IOException, TemplateException {
        EMADLGenerator gen = new EMADLGenerator();
        gen.setModelsPath( modelsDirPath );
        TaggingResolver symtab = AbstractSymtab.createSymTabAndTaggingResolver(gen.getModelsPath());
        ComponentSymbol component = symtab.<ComponentSymbol>resolve(qualifiedName, ComponentSymbol.KIND).orElse(null);

        List<String> splitName = Splitters.DOT.splitToList(qualifiedName);
        String componentName = splitName.get(splitName.size() - 1);
        String instanceName = componentName.substring(0, 1).toLowerCase() + componentName.substring(1);

        if (component == null){
            Log.error("Component with name '" + componentName + "' does not exist.");
            System.exit(1);
        }

        ExpandedComponentInstanceSymbol instance = component.getEnclosingScope().<ExpandedComponentInstanceSymbol>resolve(instanceName, ExpandedComponentInstanceSymbol.KIND).get();

        return gen.generateStrings(symtab, instance, symtab);
    }

    private String readFileFromResources(String relativePath) throws IOException{
        ClassLoader classLoader = getClass().getClassLoader();
        File file = new File(classLoader.getResource(relativePath).getFile());
        Scanner scanner = new Scanner(file);
        scanner.useDelimiter("\\Z");
        String content = scanner.next() + "\n";
        scanner.close();
        return content;
    }

    @Test
    public void testCifar10Generation() throws IOException, TemplateException {
        //generate("cifar10.Cifar10Classifier");
        Log.getFindings().clear();

        List<FileContent> fileContents = generateStrings(
                "src/test/resources/models/",
                "cifar10.Cifar10Classifier");
        assertTrue(Log.getFindings().isEmpty());

        for (FileContent fileContent : fileContents){
            switch (fileContent.getFileName()){
                case "cifar10_cifar10Classifier.h":
                    assertEquals(fileContent.getFileContent(),
                            readFileFromResources("target_code/cifar10_cifar10Classifier.h"));
                    break;
                case "CNNCreator_cifar10_cifar10Classifier_net.py":
                    assertEquals(fileContent.getFileContent(),
                            readFileFromResources("target_code/CNNCreator_cifar10_cifar10Classifier_net.py"));
                    break;
                case "CNNBufferFile.h":
                    assertEquals(fileContent.getFileContent(),
                            readFileFromResources("target_code/CNNBufferFile.h"));
                    break;
                case "CNNPredictor_cifar10_cifar10Classifier_net.h":
                    assertEquals(fileContent.getFileContent(),
                            readFileFromResources("target_code/CNNPredictor_cifar10_cifar10Classifier_net.h"));
                    break;
                case "cifar10_cifar10Classifier_net.h":
                    assertEquals(fileContent.getFileContent(),
                            readFileFromResources("target_code/cifar10_cifar10Classifier_net.h"));
                    break;
                case "CNNTranslator.h":
                    assertEquals(fileContent.getFileContent(),
                            readFileFromResources("target_code/CNNTranslator.h"));
                    break;
                case "cifar10_cifar10Classifier_calculateClass.h":
                    assertEquals(fileContent.getFileContent(),
                            readFileFromResources("target_code/cifar10_cifar10Classifier_calculateClass.h"));
                    break;
                case "CNNTrainer_cifar10_Cifar10Classifier.py":
                    assertEquals(fileContent.getFileContent(),
                            readFileFromResources("target_code/CNNTrainer_cifar10_Cifar10Classifier.py"));
                    break;
            }
        }
    }

    @Test
    public void testSimulatorGeneration() throws IOException, TemplateException {
        generate("simulator.MainController");
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testAddGeneration() throws IOException, TemplateException {
        generate("Add");
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testAlexnetGeneration() throws IOException, TemplateException {
        generate("Alexnet");
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testResNeXtGeneration() throws IOException, TemplateException {
        generate("ResNeXt50");
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testThreeInputGeneration() throws IOException, TemplateException {
        generate("ThreeInputCNN_M14");
        assertTrue(Log.getFindings().size() == 1);
    }

    @Test
    public void testMultipleOutputsGeneration() throws IOException, TemplateException {
        generate("MultipleOutputs");
        assertTrue(Log.getFindings().size() == 1);
    }

    @Test
    public void tesVGGGeneration() throws IOException, TemplateException {
        generate("VGG16");
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testMultipleInstances() throws IOException, TemplateException {
        generate("InstanceTest.MainB");
        assertTrue(Log.getFindings().isEmpty());
    }
}
