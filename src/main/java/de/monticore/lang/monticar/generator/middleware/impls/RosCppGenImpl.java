package de.monticore.lang.monticar.generator.middleware.impls;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.roscpp.GeneratorRosCpp;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.io.IOException;
import java.util.List;

//TODO: make GeneratorRosCpp implement GeneratorImpl
public class RosCppGenImpl implements GeneratorImpl {
    private String generationTargetPath;
    private GeneratorRosCpp generatorRosCpp;

    public RosCppGenImpl(){
        generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerateCMake(true);
    }

    public void setGeneratorRosCpp(GeneratorRosCpp generatorRosCpp) {
        this.generatorRosCpp = generatorRosCpp;
    }

    @Override
    public List<File> generate(ExpandedComponentInstanceSymbol componentInstanceSymbol, TaggingResolver taggingResolver) throws IOException {
        generatorRosCpp.setGenerationTargetPath(generationTargetPath);
        return generatorRosCpp.generateFiles(componentInstanceSymbol, taggingResolver);
    }

    @Override
    public void setGenerationTargetPath(String path) {
        this.generationTargetPath = path;
    }

    @Override
    public boolean willAccept(ExpandedComponentInstanceSymbol componentInstanceSymbol) {
        boolean result = componentInstanceSymbol.getPortsList().stream()
                .filter(PortSymbol::isRosPort)
                .count() > 0;
        if(!result){
            Log.warn("GeneratorRosCpp: No ROS Ports found! Ignoring component " + componentInstanceSymbol.getName());
        }

        return result;
    }
}
