/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.pythonwrapper.file.WrapperFileCreator;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.EmadlComponentTranslator;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.ComponentPortInformation;
import de.monticore.lang.monticar.generator.pythonwrapper.template.TemplateController;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import static com.google.common.base.Preconditions.checkArgument;

/**
 *
 */
public class GeneratorPythonWrapper {
    private String generationTargetPath;

    private final EmadlComponentTranslator componentTranslator;
    private final TemplateController templateController;
    private final WrapperFileCreator wrapperFileCreator;
    private ComponentPortInformation lastGeneratedComponentPortInformation;

    GeneratorPythonWrapper(EmadlComponentTranslator componentTranslator,
                           TemplateController templateController,
                           WrapperFileCreator wrapperFileCreator) {
        this.componentTranslator = componentTranslator;
        this.templateController = templateController;
        this.wrapperFileCreator = wrapperFileCreator;
    }

    public String getGenerationTargetPath() {
        return generationTargetPath;
    }

    public void setGenerationTargetPath(String generationTargetPath) {
        this.generationTargetPath = generationTargetPath;
    }

    public List<File> generateFiles(EMAComponentInstanceSymbol component) throws IOException {
        checkArgument(willAccept(component), "Component symbol is not supported");
        ComponentPortInformation componentPortInformation = componentTranslator.extractPortInformationFrom(component);
        Map<String, String> generatedStrings = templateController.processWrapper(componentPortInformation);
        wrapperFileCreator.setOutputDirectory(generationTargetPath);
        this.lastGeneratedComponentPortInformation = componentPortInformation;
        return wrapperFileCreator.createWrapperFiles(generatedStrings);
    }

    public Optional<ComponentPortInformation> getLastGeneratedComponentPortInformation() {
        return Optional.ofNullable(lastGeneratedComponentPortInformation);
    }

    public boolean willAccept(EMAComponentInstanceSymbol componentInstanceSymbol) {
        return componentTranslator.willAccept(componentInstanceSymbol);
    }
}
