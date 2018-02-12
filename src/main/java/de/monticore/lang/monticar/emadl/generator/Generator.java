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

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEmbeddedMontiArcNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.emadl._cocos.EMADLCocos;
import de.monticore.lang.monticar.emadl._symboltable.EMADLBehaviorSymbol;
import freemarker.template.TemplateException;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;


public class Generator {

    public void generate(Path modelPath, String componentName) throws IOException, TemplateException {
        ModelLoader loader = new ModelLoader(modelPath);
        ExpandedComponentInstanceSymbol instance = loader.load(componentName);

        String targetDirectory = modelPath.toString() + "/target/" + instance.getName() + "/";
        generate(instance, targetDirectory);
    }

    public void generate(ExpandedComponentInstanceSymbol instance, String targetDirectory) throws IOException{
        String componentName = instance.getComponentType().getName();
        EMADLBehaviorSymbol emadl = instance.getEnclosingScope().<EMADLBehaviorSymbol> resolve(componentName, EMADLBehaviorSymbol.KIND).get();
        ASTEmbeddedMontiArcNode astComponent = (ASTEmbeddedMontiArcNode) instance.getComponentType().getReferencedSymbol().getAstNode().get();

        EMADLCocos.createPreResolveChecker().checkAll(astComponent);

        ArchitectureSymbol architecture = emadl.resolveArchitecture();
        ConfigurationSymbol configuration = emadl.resolveConfiguration();

        EMADLCocos.createPostResolveChecker().checkAll(astComponent);

        CNNArchTemplateController archTc = new CNNArchTemplateController(architecture, Target.CPP);

        File networkFile = new File(targetDirectory + "network" + archTc.getTarget());
        networkFile.getParentFile().mkdirs();
        networkFile.createNewFile();
        FileWriter writer = new FileWriter(networkFile);

        archTc.process(writer);
        writer.close();
    }

    /*protected Path getTargetFilePath(Path modelPath, String qualifiedName) {
        List<String> list = Splitters.DOT.splitToList(qualifiedName);
        String targetDir = modelPath.toAbsolutePath().toString() + Joiners.DOT.join(list.subList(0, list.size() - 1));
        String targetName = list.get(list.size() - 1);
        return Paths.get(targetDir,
                GENERATION_PATH + targetName + getTargetFileEnding());
    }

    protected FileWriter getTargetFileWriter(Path modelPath, String targetName) throws IOException {
        File file = getTargetFilePath(modelPath, targetName).toFile();
        file.getParentFile().mkdirs();
        file.createNewFile();
        return new FileWriter(file);
    }*/


    public static void main(String[] args) throws IOException, TemplateException {
        if(args.length != 2){
            System.err.println("Two argument are required (directory path and component name (without extension))");
        }
        Path modelPath = Paths.get(args[0]).toAbsolutePath();
        String qualifiedName = args[1];

        Generator generator = new Generator();
        generator.generate(modelPath, qualifiedName);
    }
}
