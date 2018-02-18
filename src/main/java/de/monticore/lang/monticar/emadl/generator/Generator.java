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

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.math.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.emadl._cocos.EMADLCocos;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.*;
import de.monticore.lang.monticar.generator.cpp.converter.MathConverter;
import de.monticore.lang.monticar.generator.cpp.converter.TypeConverter;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.Splitters;
import de.se_rwth.commons.logging.Log;
import freemarker.template.TemplateException;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;


public class Generator {

    private GeneratorCPP emamGen;
    private Target targetLanguage;

    public Generator() {
        targetLanguage = Target.CPP;
        emamGen = new GeneratorCPP();
        //MathConverter.curBackend = new ArmadilloBackend();
    }

    public GeneratorCPP getEmamGen() {
        return emamGen;
    }

    public Target getTargetLanguage() {
        return targetLanguage;
    }

    public List<FileContent> generateStrings(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol componentInstanceSymbol, Scope symtab){
        List<FileContent> fileContents = new ArrayList<>();
        ASTComponent astComponent = (ASTComponent) componentInstanceSymbol.getComponentType().getReferencedSymbol().getAstNode().get();
        EMADLCocos.createChecker().checkAll(astComponent);

        Optional<ArchitectureSymbol> architecture = astComponent.getSpannedScope().get().resolve("", ArchitectureSymbol.KIND);
        Optional<MathStatementsSymbol> mathStatements = astComponent.getSpannedScope().get().resolve("MathStatements", MathStatementsSymbol.KIND);

        if (architecture.isPresent()){
            fileContents.addAll(generateCNNImplementation(componentInstanceSymbol, architecture.get()));
        }
        else if (mathStatements.isPresent()){
            fileContents.addAll(generateMathImplementation(taggingResolver, componentInstanceSymbol, mathStatements.get()));
        }
        else {
            fileContents.addAll(generateSubComponents(taggingResolver, componentInstanceSymbol, symtab));
        }

        if (MathConverter.curBackend.getBackendName().equals("OctaveBackend"))
            fileContents.add(OctaveHelper.getOctaveHelperFileContent());
        if (MathConverter.curBackend.getBackendName().equals("ArmadilloBackend"))
            fileContents.add(ArmadilloHelper.getArmadilloHelperFileContent());

        if (emamGen.shouldGenerateMainClass()) {
            //fileContents.add(emamGen.getMainClassFileContent(componentInstanceSymbol, fileContents.get(0)));
        } else if (emamGen.shouldGenerateSimulatorInterface()) {
            fileContents.addAll(SimulatorIntegrationHelper.getSimulatorIntegrationHelperFileContent());
        }

        return fileContents;
    }

    public List<FileContent> generateCNNImplementation(ExpandedComponentInstanceSymbol instance, ArchitectureSymbol architecture){
        CNNArchTemplateController archTc = new CNNArchTemplateController(architecture, targetLanguage);
        FileContent fileContent = new FileContent(archTc.process(), instance);
        return Collections.singletonList(fileContent);
    }

    public List<FileContent> generateMathImplementation(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol componentSymbol, MathStatementsSymbol mathStatementsSymbol){
        FileContent fileContent = new FileContent(emamGen.generateString(taggingResolver, componentSymbol, mathStatementsSymbol), componentSymbol);
        return Collections.singletonList(fileContent);
    }

    public List<FileContent> generateSubComponents(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol componentInstanceSymbol, Scope symtab){
        List<FileContent> fileContents = new ArrayList<>();
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
                fileContents.addAll(generateStrings(taggingResolver, instanceSymbol, symtab));
            }
        }

        return fileContents;
    }

    public List<File> generateFiles(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol componentSymbol, Scope symtab) throws IOException {
        List<FileContent> fileContents = generateStrings(taggingResolver, componentSymbol, symtab);
        TypesGeneratorCPP tg = new TypesGeneratorCPP();
        fileContents.addAll(tg.generateTypes(TypeConverter.getTypeSymbols()));
        /*if (isGenerateTests()) {
            TestsGeneratorCPP g = new TestsGeneratorCPP(this);
            fileContents.addAll(g.generateStreamTests(symtab));
        }*/
        //System.out.println(fileContents);
        if (emamGen.getGenerationTargetPath().charAt(emamGen.getGenerationTargetPath().length() - 1) != '/') {
            emamGen.setGenerationTargetPath(emamGen.getGenerationTargetPath() + "/");
        }
        List<File> files = new ArrayList<>();
        for (FileContent fileContent : fileContents) {
            files.add(emamGen.generateFile(fileContent));
        }
        return files;
    }

    public void generate(Path modelPath, String qualifiedName) throws IOException, TemplateException {
        TaggingResolver symtab = AbstractSymtab.createSymTabAndTaggingResolver("src/test/resources");
        ComponentSymbol component = symtab.<ComponentSymbol>resolve(qualifiedName, ComponentSymbol.KIND).orElse(null);

        List<String> splitName = Splitters.DOT.splitToList(qualifiedName);
        String componentName = splitName.get(splitName.size() - 1);
        String instanceName = componentName.substring(0, 1).toLowerCase() + componentName.substring(1);
        ExpandedComponentInstanceSymbol instance = component.getEnclosingScope().<ExpandedComponentInstanceSymbol>resolve(instanceName, ExpandedComponentInstanceSymbol.KIND).get();

        generateFiles(symtab, instance, symtab);
    }


    public static void main(String[] args) throws IOException, TemplateException {
        if(args.length < 2){
            System.err.println("Two argument are required (directory path and component name (without extension))");
        }
        Path modelPath = Paths.get(args[0]).toAbsolutePath();
        String qualifiedName = args[1];

        Generator generator = new Generator();
        if (args.length >= 3){
            generator.targetLanguage = Target.fromString(args[2]);
        }

        generator.generate(modelPath, qualifiedName);
    }
}
