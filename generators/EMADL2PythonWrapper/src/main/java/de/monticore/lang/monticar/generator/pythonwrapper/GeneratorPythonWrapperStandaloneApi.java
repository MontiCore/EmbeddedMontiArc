/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.pythonwrapper.building.PythonModuleBuilder;
import de.monticore.lang.monticar.generator.pythonwrapper.building.PythonModuleBuildingException;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.ComponentPortInformation;
import de.se_rwth.commons.logging.Log;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

/**
 *
 */
public class GeneratorPythonWrapperStandaloneApi {
    private final PythonModuleBuilder pythonModuleBuilder;

    public GeneratorPythonWrapperStandaloneApi() {
        pythonModuleBuilder = new PythonModuleBuilder();

    }

    public ComponentPortInformation generate(final EMAComponentInstanceSymbol componentInstanceSymbol,
                                             final String outputPath) {
        GeneratorPythonWrapper generatorPythonWrapper = new GeneratorPythonWrapperFactory().create();
        generatorPythonWrapper.setGenerationTargetPath(outputPath);
        try {
            generatorPythonWrapper.generateFiles(componentInstanceSymbol);
        } catch (IOException e) {
            Log.error("Error while generating files: " + e.getMessage());
            System.exit(1);
        }

        assert generatorPythonWrapper.getLastGeneratedComponentPortInformation().isPresent()
                : "State error: No port information available";
        return generatorPythonWrapper.getLastGeneratedComponentPortInformation().get();
    }

    public ComponentPortInformation generateAndTryBuilding(final EMAComponentInstanceSymbol componentInstanceSymbol,
                                                           final String generationOutputPath,
                                                           final String moduleOutputPath) {
        ComponentPortInformation componentPortInformation = generate(componentInstanceSymbol, generationOutputPath);
        if (this.checkIfPythonModuleBuildAvailable()) {
            try {
                buildPythonModule(Paths.get(generationOutputPath),
                        componentPortInformation.getComponentName(),
                        Paths.get(moduleOutputPath));
            } catch (PythonModuleBuildingException e) {
                Log.warn("Cannot build python module: " + e.getMessage());
            }
        } else {
            Log.warn("Cannot build python module: OS not supported");
        }
        return componentPortInformation;
    }

    public boolean checkIfPythonModuleBuildAvailable() {
        return pythonModuleBuilder.checkPythonModuleBuildAvailable();
    }

    public void buildPythonModule(final Path sourceDirectory,
                                  final String componentName,
                                  final Path outputDirectory) throws PythonModuleBuildingException {
        pythonModuleBuilder.buildPythonModule(sourceDirectory, componentName, outputDirectory);
    }
}
