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
package de.monticore.lang.monticar.cnnarch.caffe2generator;

import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.cnntrain._cocos.CNNTrainCocos;
import de.monticore.lang.monticar.cnntrain._symboltable.CNNTrainCompilationUnitSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.CNNTrainLanguage;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.monticore.symboltable.GlobalScope;
import de.se_rwth.commons.logging.Log;
import freemarker.template.TemplateException;
import org.junit.Before;
import org.junit.Test;

import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.*;

import static junit.framework.TestCase.assertTrue;

public class GenerationTest extends AbstractSymtabTest{

    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void testCifar10Classifier() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/valid_tests", "-r", "CifarClassifierNetwork", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2Caffe2Cli.main(args);
        assertTrue(Log.getFindings().isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                "CNNCreator_CifarClassifierNetwork.py",
                "CNNPredictor_CifarClassifierNetwork.h",
                "execute_CifarClassifierNetwork",
                "CNNBufferFile.h"));
    }

    @Test
    public void testAlexnetGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/architectures", "-r", "Alexnet", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2Caffe2Cli.main(args);
        assertTrue(Log.getFindings().isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CNNCreator_Alexnet.py",
                        "CNNPredictor_Alexnet.h",
                        "execute_Alexnet"));
    }

    @Test
    public void testGeneratorVGG16() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/architectures", "-r", "VGG16", "-o", "./target/generated-sources-cnnarch/"};
        CNNArch2Caffe2Cli.main(args);
        assertTrue(Log.getFindings().isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CNNCreator_VGG16.py",
                        "CNNPredictor_VGG16.h",
                        "execute_VGG16"));
    }


    @Test
    public void testThreeInputCNNGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/architectures", "-r", "ThreeInputCNN_M14"};
        CNNArch2Caffe2Cli.main(args);
        assertTrue(Log.getFindings().size() == 1);
    }

    @Test
    public void testResNeXtGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();;
        String[] args = {"-m", "src/test/resources/architectures", "-r", "ResNeXt50"};
        CNNArch2Caffe2Cli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testMultipleOutputs() throws IOException, TemplateException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/valid_tests", "-r", "MultipleOutputs"};
        CNNArch2Caffe2Cli.main(args);
        assertTrue(Log.getFindings().size() == 3);
    }

    @Test
    public void testCNNTrainerGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        List<ConfigurationSymbol> configurations = new ArrayList<>();
        List<String> instanceNames = Arrays.asList("main_net1", "main_net2");

        final ModelPath mp = new ModelPath(Paths.get("src/test/resources/valid_tests"));
        GlobalScope scope = new GlobalScope(mp, new CNNTrainLanguage());

        CNNTrainCompilationUnitSymbol compilationUnit = scope.<CNNTrainCompilationUnitSymbol>
                resolve("Network1", CNNTrainCompilationUnitSymbol.KIND).get();
        CNNTrainCocos.checkAll(compilationUnit);
        configurations.add(compilationUnit.getConfiguration());

        compilationUnit = scope.<CNNTrainCompilationUnitSymbol>
                resolve("Network2", CNNTrainCompilationUnitSymbol.KIND).get();
        CNNTrainCocos.checkAll(compilationUnit);
        configurations.add(compilationUnit.getConfiguration());

        CNNArch2Caffe2 generator = new CNNArch2Caffe2();
        Map<String,String> trainerMap = generator.generateTrainer(configurations, instanceNames, "main");

        for (String fileName : trainerMap.keySet()){
            FileWriter writer = new FileWriter(generator.getGenerationTargetPath() + fileName);
            writer.write(trainerMap.get(fileName));
            writer.close();
        }

        assertTrue(Log.getFindings().isEmpty());
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CNNTrainer_main.py"));
    }

    @Test
    public void testFullCfgGeneration() throws IOException, TemplateException {
        Log.getFindings().clear();
        List<ConfigurationSymbol> configurations = new ArrayList<>();
        List<String> instanceName = Arrays.asList("main_net1", "main_net2");

        final ModelPath mp = new ModelPath(Paths.get("src/test/resources/valid_tests"));
        GlobalScope scope = new GlobalScope(mp, new CNNTrainLanguage());

        CNNTrainCompilationUnitSymbol compilationUnit = scope.<CNNTrainCompilationUnitSymbol>
                resolve("FullConfig", CNNTrainCompilationUnitSymbol.KIND).get();
        CNNTrainCocos.checkAll(compilationUnit);
        configurations.add(compilationUnit.getConfiguration());

        compilationUnit = scope.<CNNTrainCompilationUnitSymbol>
                resolve("FullConfig2", CNNTrainCompilationUnitSymbol.KIND).get();
        CNNTrainCocos.checkAll(compilationUnit);
        configurations.add(compilationUnit.getConfiguration());

        CNNArch2Caffe2 generator = new CNNArch2Caffe2();
        Map<String,String> trainerMap = generator.generateTrainer(configurations, instanceName, "mainFull");

        for (String fileName : trainerMap.keySet()){
            FileWriter writer = new FileWriter(generator.getGenerationTargetPath() + fileName);
            writer.write(trainerMap.get(fileName));
            writer.close();
        }

        assertTrue(Log.getFindings().isEmpty());
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CNNTrainer_mainFull.py"));
    }

    @Test
    public void testSimpleCfgGeneration() throws IOException {
        Log.getFindings().clear();
        List<ConfigurationSymbol> configurations = new ArrayList<>();
        List<String> instanceName = Arrays.asList("main_net1", "main_net2");

        final ModelPath mp = new ModelPath(Paths.get("src/test/resources/valid_tests"));
        GlobalScope scope = new GlobalScope(mp, new CNNTrainLanguage());

        CNNTrainCompilationUnitSymbol compilationUnit = scope.<CNNTrainCompilationUnitSymbol>
                resolve("SimpleConfig1", CNNTrainCompilationUnitSymbol.KIND).get();
        CNNTrainCocos.checkAll(compilationUnit);
        configurations.add(compilationUnit.getConfiguration());

        compilationUnit = scope.<CNNTrainCompilationUnitSymbol>
                resolve("SimpleConfig2", CNNTrainCompilationUnitSymbol.KIND).get();
        CNNTrainCocos.checkAll(compilationUnit);
        configurations.add(compilationUnit.getConfiguration());

        CNNArch2Caffe2 generator = new CNNArch2Caffe2();
        Map<String,String> trainerMap = generator.generateTrainer(configurations, instanceName, "mainSimple");

        for (String fileName : trainerMap.keySet()){
            FileWriter writer = new FileWriter(generator.getGenerationTargetPath() + fileName);
            writer.write(trainerMap.get(fileName));
            writer.close();
        }

        assertTrue(Log.getFindings().isEmpty());
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CNNTrainer_mainSimple.py"));
    }

    @Test
    public void testEmptyCfgGeneration() throws IOException {
        Log.getFindings().clear();
        List<ConfigurationSymbol> configurations = new ArrayList<>();
        List<String> instanceName = Arrays.asList("main_net1");

        final ModelPath mp = new ModelPath(Paths.get("src/test/resources/valid_tests"));
        GlobalScope scope = new GlobalScope(mp, new CNNTrainLanguage());

        CNNTrainCompilationUnitSymbol compilationUnit = scope.<CNNTrainCompilationUnitSymbol>
                resolve("EmptyConfig", CNNTrainCompilationUnitSymbol.KIND).get();
        CNNTrainCocos.checkAll(compilationUnit);
        configurations.add(compilationUnit.getConfiguration());

        CNNArch2Caffe2 generator = new CNNArch2Caffe2();
        Map<String,String> trainerMap = generator.generateTrainer(configurations, instanceName, "mainEmpty");

        for (String fileName : trainerMap.keySet()){
            FileWriter writer = new FileWriter(generator.getGenerationTargetPath() + fileName);
            writer.write(trainerMap.get(fileName));
            writer.close();
        }

        assertTrue(Log.getFindings().isEmpty());
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CNNTrainer_mainEmpty.py"));
    }


    @Test
    public void testCMakeGeneration() {
        Log.getFindings().clear();
        String rootModelName = "alexnet";
        CNNArch2Caffe2 generator = new CNNArch2Caffe2();
        generator.setGenerationTargetPath("./target/generated-sources-cnnarch");
        generator.generateCMake(rootModelName);

        assertTrue(Log.getFindings().isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code"),
                Arrays.asList(
                        "CMakeLists.txt"));

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch/cmake"),
                Paths.get("./src/test/resources/target_code/cmake"),
                Arrays.asList(
                        "FindArmadillo.cmake"));
    }

}
