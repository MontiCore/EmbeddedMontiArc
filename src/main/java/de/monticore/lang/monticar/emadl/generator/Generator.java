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

import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.math.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch.generator.CNNArchGenerator;
import de.monticore.lang.monticar.cnntrain._symboltable.CNNTrainLanguage;
import de.monticore.lang.monticar.cnntrain.generator.CNNTrainGenerator;
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
import freemarker.template.Template;
import freemarker.template.TemplateException;

import java.io.IOException;
import java.io.StringWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;


public class Generator {

    public static final String CNN_HELPER = "CNNTranslator";
    public static final String CNN_TRAINER = "CNNTrainer";

    private GeneratorCPP emamGen;

    public Generator() {
        emamGen = new GeneratorCPP();
        emamGen.useArmadilloBackend();
        emamGen.setGenerationTargetPath("./target/generated-sources-emadl/");
    }

    private String modelsPath;

    public String getModelsPath() {
        return modelsPath;
    }

    public void setModelsPath(String modelsPath) {
        this.modelsPath = modelsPath;
    }

    public void setGenerationTargetPath(String generationTargetPath){
        getEmamGen().setGenerationTargetPath(generationTargetPath);
    }

    public String getGenerationTargetPath(){
        return getEmamGen().getGenerationTargetPath();
    }

    public GeneratorCPP getEmamGen() {
        return emamGen;
    }

    public List<FileContent> generateStrings(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol componentInstanceSymbol, Scope symtab){
        List<FileContent> fileContents = new ArrayList<>();
        Set<ExpandedComponentInstanceSymbol> allInstances = new HashSet<>();

        generateStrings(fileContents, allInstances, taggingResolver, componentInstanceSymbol, symtab);

        fileContents.add(generateCNNTrainer(allInstances, componentInstanceSymbol.getComponentType().getFullName().replaceAll("\\.", "_")));
        fileContents.add(ArmadilloHelper.getArmadilloHelperFileContent());
        TypesGeneratorCPP tg = new TypesGeneratorCPP();
        fileContents.addAll(tg.generateTypes(TypeConverter.getTypeSymbols()));

        if (emamGen.shouldGenerateMainClass()) {
            //fileContents.add(emamGen.getMainClassFileContent(componentInstanceSymbol, fileContents.get(0)));
        } else if (emamGen.shouldGenerateSimulatorInterface()) {
            fileContents.addAll(SimulatorIntegrationHelper.getSimulatorIntegrationHelperFileContent());
        }

        return fileContents;
    }

    protected void generateStrings(List<FileContent> fileContents,
                                Set<ExpandedComponentInstanceSymbol> allInstances,
                                TaggingResolver taggingResolver,
                                ExpandedComponentInstanceSymbol componentInstanceSymbol,
                                Scope symtab){
        allInstances.add(componentInstanceSymbol);
        ASTComponent astComponent = (ASTComponent) componentInstanceSymbol.getComponentType().getReferencedSymbol().getAstNode().get();
        EMADLCocos.createChecker().checkAll(astComponent);

        Optional<ArchitectureSymbol> architecture = astComponent.getSpannedScope().get().resolve("", ArchitectureSymbol.KIND);
        Optional<MathStatementsSymbol> mathStatements = astComponent.getSpannedScope().get().resolve("MathStatements", MathStatementsSymbol.KIND);

        if (architecture.isPresent()){
            generateCNN(fileContents, taggingResolver, componentInstanceSymbol, architecture.get().resolve());
        }
        else if (mathStatements.isPresent()){
            generateMathComponent(fileContents, taggingResolver, componentInstanceSymbol, mathStatements.get());
        }
        else {
            generateSubComponents(fileContents, allInstances, taggingResolver, componentInstanceSymbol, symtab);
        }
    }

    public void generateCNN(List<FileContent> fileContents, TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol instance, ArchitectureSymbol architecture){
        CNNArchGenerator cnnArchGenerator = new CNNArchGenerator();
        Map<String,String> contentMap = cnnArchGenerator.generateStrings(architecture);
        String fullName = instance.getComponentType().getReferencedSymbol().getFullName();

        //get the components execute method
        String executeKey = "execute_" + fullName.replaceAll("\\.", "_");
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
        fileContents.add(new FileContent(processTemplate(new HashMap<>(), CNN_HELPER), CNN_HELPER + ".h"));
    }

    protected String transformComponent(String component, String predictorClassName, String executeMethod){
        String networkVariableName = "cnn_";
        //insert includes
        component = component.replaceFirst("using namespace",
                "#include \"" + predictorClassName + ".h" + "\"\n" +
                        "#include \"" + CNN_HELPER + ".h" + "\"\n" +
                        "using namespace");

        //insert network attribute
        component = component.replaceFirst("public:",
                "public:\n" + predictorClassName + " " + networkVariableName + ";");

        //insert attribute initialization
        component = component.replaceFirst("void init\\(\\)\\s\\{",
                "void init()\n{\n" + networkVariableName + "=" + predictorClassName + "();");

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
                generateStrings(fileContents, allInstances, taggingResolver, instanceSymbol, symtab);
            }
        }
    }

    public FileContent generateCNNTrainer(Set<ExpandedComponentInstanceSymbol> allInstances, String mainComponentName){
        List<ExpandedComponentInstanceSymbol> cnnInstances = new ArrayList<>();
        List<String> trainParams = new ArrayList<>();
        Set<String> componentNames = new HashSet<>();
        for (ExpandedComponentInstanceSymbol componentInstance : allInstances){
            ComponentSymbol component = componentInstance.getComponentType().getReferencedSymbol();
            Optional<ArchitectureSymbol> architecture = component.getSpannedScope().resolve("", ArchitectureSymbol.KIND);

            if (architecture.isPresent()){

                String fileContent = getTrainingParamsForComponent(mainComponentName);
                if (!fileContent.isEmpty()) {
                    trainParams.add(fileContent);
                }

                cnnInstances.add(componentInstance);
                componentNames.add(component.getFullName());

            }
        }
        Map<String, Object> ftlContext = new HashMap<>();
        ftlContext.put("instances", cnnInstances);
        ftlContext.put("componentNames", componentNames);
        ftlContext.put("trainParams", trainParams);
        return new FileContent(processTemplate(ftlContext, CNN_TRAINER), CNN_TRAINER + "_" + mainComponentName + ".py");
    }

    private String getTrainingParamsForComponent(String mainComponentName) {
        String configFilename = mainComponentName + "Config";
        if (!Files.exists(Paths.get( getModelsPath() + configFilename + ".cnnt"))) {
            return "";
        }

        CNNTrainGenerator cnnTrainGenerator =  new CNNTrainGenerator();
        final ModelPath mp = new ModelPath(Paths.get(getModelsPath()));
        GlobalScope trainScope = new GlobalScope(mp, new CNNTrainLanguage());
        Map.Entry<String, String> fileContents = cnnTrainGenerator.generateFileContent( trainScope, configFilename );
        return fileContents.getValue();
    }

    public void generateFiles(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol componentSymbol, Scope symtab) throws IOException {
        List<FileContent> fileContents = generateStrings(taggingResolver, componentSymbol, symtab);

        for (FileContent fileContent : fileContents) {
            emamGen.generateFile(fileContent);
        }
    }

    public void generate(String modelPath, String qualifiedName) throws IOException, TemplateException {
        setModelsPath( modelPath );
        TaggingResolver symtab = AbstractSymtab.createSymTabAndTaggingResolver(getModelsPath());
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

    protected String processTemplate(Map<String, Object> ftlContext, String templateNameWithoutEnding){
        StringWriter writer = new StringWriter();
        String templateName = templateNameWithoutEnding + ".ftl";
        try{
            Template template = TemplateConfiguration.get().getTemplate(templateName);
            template.process(ftlContext, writer);
        }
        catch (IOException e) {
            Log.error("Freemarker could not find template " + templateName + " :\n" + e.getMessage());
            System.exit(1);
        }
        catch (TemplateException e){
            Log.error("An exception occured in template " + templateName + " :\n" + e.getMessage());
            System.exit(1);
        }
        return writer.toString();
    }

}
