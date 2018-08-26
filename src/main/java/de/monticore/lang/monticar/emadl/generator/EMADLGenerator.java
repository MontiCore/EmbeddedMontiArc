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
package de.monticore.lang.monticar.emadl.generator;

import com.google.common.base.Joiner;
import com.google.common.base.Splitter;
import com.google.common.base.Charsets;
import com.google.common.io.Resources;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.monticar.cnnarch.CNNArchGenerator;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnntrain._cocos.CNNTrainCocos;
import de.monticore.lang.monticar.cnntrain._symboltable.CNNTrainCompilationUnitSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.CNNTrainLanguage;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.emadl._cocos.EMADLCocos;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.ArmadilloHelper;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.cpp.SimulatorIntegrationHelper;
import de.monticore.lang.monticar.generator.cpp.TypesGeneratorCPP;
import de.monticore.lang.monticar.generator.cpp.converter.TypeConverter;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.Splitters;
import de.se_rwth.commons.logging.Log;
import freemarker.template.TemplateException;

import java.io.*;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;


public class EMADLGenerator {

    private GeneratorCPP emamGen;
    private CNNArchGenerator cnnArchGenerator;

    private String modelsPath;


    public EMADLGenerator(Backend backend) {
        emamGen = new GeneratorCPP();
        emamGen.useArmadilloBackend();
        emamGen.setGenerationTargetPath("./target/generated-sources-emadl/");
        cnnArchGenerator = backend.getGenerator();
    }

    public String getModelsPath() {
        return modelsPath;
    }

    public void setModelsPath(String modelsPath) {
        if (!(modelsPath.substring(modelsPath.length() - 1).equals("/"))){
            this.modelsPath = modelsPath + "/";
        }
        else {
            this.modelsPath = modelsPath;
        }
    }

    public void setGenerationTargetPath(String generationTargetPath){
        if (!(generationTargetPath.substring(generationTargetPath.length() - 1).equals("/"))){
            getEmamGen().setGenerationTargetPath(generationTargetPath + "/");
        }
        else {
            getEmamGen().setGenerationTargetPath(generationTargetPath);
        }
    }

    public String getGenerationTargetPath(){
        return getEmamGen().getGenerationTargetPath();
    }

    public GeneratorCPP getEmamGen() {
        return emamGen;
    }

    public void generate(String modelPath, String qualifiedName) throws IOException, TemplateException {
        setModelsPath( modelPath );
        TaggingResolver symtab = EMADLAbstractSymtab.createSymTabAndTaggingResolver(getModelsPath());
        ComponentSymbol component = symtab.<ComponentSymbol>resolve(qualifiedName, ComponentSymbol.KIND).orElse(null);

        List<String> splitName = Splitters.DOT.splitToList(qualifiedName);
        String componentName = splitName.get(splitName.size() - 1);
        String instanceName = componentName.substring(0, 1).toLowerCase() + componentName.substring(1);

        if (component == null){
            Log.error("Component with name '" + componentName + "' does not exist.");
            System.exit(1);
        }

        ExpandedComponentInstanceSymbol instance = component.getEnclosingScope().<ExpandedComponentInstanceSymbol>resolve(instanceName, ExpandedComponentInstanceSymbol.KIND).get();

        generateFiles(symtab, instance, symtab);
    }

    public void generateFiles(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol componentSymbol, Scope symtab) throws IOException {
        List<FileContent> fileContents = generateStrings(taggingResolver, componentSymbol, symtab);

        for (FileContent fileContent : fileContents) {
            emamGen.generateFile(fileContent);
        }
    }

    public List<FileContent> generateStrings(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol componentInstanceSymbol, Scope symtab){
        List<FileContent> fileContents = new ArrayList<>();
        Set<ExpandedComponentInstanceSymbol> allInstances = new HashSet<>();

        generateComponent(fileContents, allInstances, taggingResolver, componentInstanceSymbol, symtab);

        fileContents.addAll(generateCNNTrainer(allInstances, componentInstanceSymbol.getComponentType().getFullName().replaceAll("\\.", "_")));
        fileContents.add(ArmadilloHelper.getArmadilloHelperFileContent());
        TypesGeneratorCPP tg = new TypesGeneratorCPP();
        fileContents.addAll(tg.generateTypes(TypeConverter.getTypeSymbols()));

        if (emamGen.shouldGenerateMainClass()) {
            //fileContents.add(emamGen.getMainClassFileContent(componentInstanceSymbol, fileContents.get(0)));
        } else if (emamGen.shouldGenerateSimulatorInterface()) {
            fileContents.addAll(SimulatorIntegrationHelper.getSimulatorIntegrationHelperFileContent());
        }

        fixArmadilloImports(fileContents);

        return fileContents;
    }

    protected void generateComponent(List<FileContent> fileContents,
                                     Set<ExpandedComponentInstanceSymbol> allInstances,
                                     TaggingResolver taggingResolver,
                                     ExpandedComponentInstanceSymbol componentInstanceSymbol,
                                     Scope symtab){
        allInstances.add(componentInstanceSymbol);
        ComponentSymbol componentSymbol = componentInstanceSymbol.getComponentType().getReferencedSymbol();

        /* remove the following two lines if the component symbol full name bug with generic variables is fixed */
        componentSymbol.setFullName(null);
        componentSymbol.getFullName();
        /* */

        Optional<ArchitectureSymbol> architecture = componentInstanceSymbol.getSpannedScope().resolve("", ArchitectureSymbol.KIND);
        Optional<MathStatementsSymbol> mathStatements = componentSymbol.getSpannedScope().resolve("MathStatements", MathStatementsSymbol.KIND);

        EMADLCocos.checkAll(componentInstanceSymbol);

        if (architecture.isPresent()){
            generateCNN(fileContents, taggingResolver, componentInstanceSymbol, architecture.get());
        }
        else if (mathStatements.isPresent()){
            generateMathComponent(fileContents, taggingResolver, componentInstanceSymbol, mathStatements.get());
        }
        else {
            generateSubComponents(fileContents, allInstances, taggingResolver, componentInstanceSymbol, symtab);
        }
    }

    private void fixArmadilloImports(List<FileContent> fileContents){
        for (FileContent fileContent : fileContents){
            fileContent.setFileContent(fileContent.getFileContent()
                    .replaceFirst("#include \"armadillo.h\"",
                            "#include \"armadillo\""));
        }
    }

    public void generateCNN(List<FileContent> fileContents, TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol instance, ArchitectureSymbol architecture){
        Map<String,String> contentMap = cnnArchGenerator.generateStrings(architecture);
        String fullName = instance.getFullName().replaceAll("\\.", "_");

        //get the components execute method
        String executeKey = "execute_" + fullName;
        String executeMethod = contentMap.get(executeKey);
        if (executeMethod == null){
            throw new IllegalStateException("execute method of " + fullName + " not found");
        }
        contentMap.remove(executeKey);

        String component = emamGen.generateString(taggingResolver, instance, (MathStatementsSymbol) null);
        FileContent componentFileContent = new FileContent(
                transformComponent(component, "CNNPredictor_" + fullName, executeMethod),
                instance);

        for (String fileName : contentMap.keySet()){
            fileContents.add(new FileContent(contentMap.get(fileName), fileName));
        }
        fileContents.add(componentFileContent);
        fileContents.add(new FileContent(readResource("CNNTranslator.h", Charsets.UTF_8), "CNNTranslator.h"));
    }

    protected String transformComponent(String component, String predictorClassName, String executeMethod){
        String networkVariableName = "_cnn_";

        //insert includes
        component = component.replaceFirst("using namespace",
                "#include \"" + predictorClassName + ".h" + "\"\n" +
                        "#include \"CNNTranslator.h\"\n" +
                        "using namespace");

        //insert network attribute
        component = component.replaceFirst("public:",
                "public:\n" + predictorClassName + " " + networkVariableName + ";");

        //insert execute method
        component = component.replaceFirst("void execute\\(\\)\\s\\{\\s\\}",
                "void execute(){\n" + executeMethod + "\n}");
        return component;
    }

    public void generateMathComponent(List<FileContent> fileContents, TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol componentSymbol, MathStatementsSymbol mathStatementsSymbol){
        fileContents.add(new FileContent(
                emamGen.generateString(taggingResolver, componentSymbol, mathStatementsSymbol),
                componentSymbol));
    }

    public void generateSubComponents(List<FileContent> fileContents, Set<ExpandedComponentInstanceSymbol> allInstances, TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol componentInstanceSymbol, Scope symtab){
        fileContents.add(new FileContent(emamGen.generateString(taggingResolver, componentInstanceSymbol, (MathStatementsSymbol) null), componentInstanceSymbol));
        String lastNameWithoutArrayPart = "";
        for (ExpandedComponentInstanceSymbol instanceSymbol : componentInstanceSymbol.getSubComponents()) {
            int arrayBracketIndex = instanceSymbol.getName().indexOf("[");
            boolean generateComponentInstance = true;
            if (arrayBracketIndex != -1) {
                generateComponentInstance = !instanceSymbol.getName().substring(0, arrayBracketIndex).equals(lastNameWithoutArrayPart);
                lastNameWithoutArrayPart = instanceSymbol.getName().substring(0, arrayBracketIndex);
                Log.info(lastNameWithoutArrayPart, "Without:");
                Log.info(generateComponentInstance + "", "Bool:");
            }
            if (generateComponentInstance) {
                generateComponent(fileContents, allInstances, taggingResolver, instanceSymbol, symtab);
            }
        }
    }

    public List<FileContent> generateCNNTrainer(Set<ExpandedComponentInstanceSymbol> allInstances, String mainComponentName) {
        List<String> cnnInstanceNames = new ArrayList<>();
        List<ConfigurationSymbol> configurations = new ArrayList<>();
        for (ExpandedComponentInstanceSymbol componentInstance : allInstances) {
            ComponentSymbol component = componentInstance.getComponentType().getReferencedSymbol();
            Optional<ArchitectureSymbol> architecture = component.getSpannedScope().resolve("", ArchitectureSymbol.KIND);

            if (architecture.isPresent()) {
                ConfigurationSymbol configuration = getTrainingConfiguration(mainComponentName, component, componentInstance);
                configurations.add(configuration);
                cnnInstanceNames.add(componentInstance.getFullName().replaceAll("\\.", "_"));
            }
        }
        List<FileContent> fileContents = new ArrayList<>();
        Map<String, String> fileContentMap =  cnnArchGenerator.generateTrainer(configurations, cnnInstanceNames, mainComponentName);
        for (String fileName : fileContentMap.keySet()){
            fileContents.add(new FileContent(fileContentMap.get(fileName), fileName));
        }
        return fileContents;
    }

    public ConfigurationSymbol getTrainingConfiguration(String mainComponentName, ComponentSymbol component, ExpandedComponentInstanceSymbol instance) {
        String configFilename;
        String mainComponentConfigFilename = mainComponentName.replaceAll("\\.", "/");
        String componentConfigFilename = component.getFullName().replaceAll("\\.", "/");
        String instanceConfigFilename = component.getFullName().replaceAll("\\.", "/") + "_"  + instance.getName();
        if (Files.exists(Paths.get( getModelsPath() + instanceConfigFilename + ".cnnt"))) {
            configFilename = instanceConfigFilename;
        }
        else if (Files.exists(Paths.get( getModelsPath() + componentConfigFilename + ".cnnt"))){
            configFilename = componentConfigFilename;
        }
        else if (Files.exists(Paths.get( getModelsPath() + mainComponentConfigFilename + ".cnnt"))){
            configFilename = mainComponentConfigFilename;
        }
        else{
            Log.error("Missing configuration file. " +
                    "Could not find a file with any of the following names (only one needed): '"
                    + getModelsPath() + instanceConfigFilename + ".cnnt', '"
                    + getModelsPath() + componentConfigFilename + ".cnnt', '"
                    + getModelsPath() + mainComponentConfigFilename + ".cnnt'." +
                    " These files denote respectively the configuration for the single instance, the component or the whole system.");
            return null;
        }

        //should be removed when CNNTrain supports packages
        List<String> names = Splitter.on("/").splitToList(configFilename);
        configFilename = names.get(names.size()-1);
        Path modelPath = Paths.get(getModelsPath() + Joiner.on("/").join(names.subList(0,names.size()-1)));
        //

        //CNNTrainGenerator cnnTrainGenerator =  new CNNTrainGenerator(); //No need of cnnTrainGenerator since cnnArchGenerator can also generateTrainer()
        final ModelPath mp = new ModelPath(modelPath);
        GlobalScope trainScope = new GlobalScope(mp, new CNNTrainLanguage());
        Optional<CNNTrainCompilationUnitSymbol> compilationUnit = trainScope.resolve(configFilename, CNNTrainCompilationUnitSymbol.KIND);
        if (!compilationUnit.isPresent()){
            Log.error("CNNTrainCompilationUnitSymbol is empty. Could not resolve configuration " + configFilename);
            System.exit(1);
        }
        CNNTrainCocos.checkAll(compilationUnit.get());
        ConfigurationSymbol configuration = compilationUnit.get().getConfiguration();

        return configuration;
    }

    public String readResource(final String fileName, Charset charset) {
        try {
            return Resources.toString(Resources.getResource(fileName), charset);

        } catch (IllegalArgumentException e) {
            System.err.println("Resource file " + fileName + " not found");
            System.exit(1);
            return null;
        } catch (IOException e) {
            System.err.println("IO Error occurred");
            System.exit(1);
            return null;
        }
    }
}
