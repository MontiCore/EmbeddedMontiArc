/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.util;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

import java.util.HashMap;
import java.util.Map;

/**
 *
 */
public class ComponentInstanceSymbolLibrary {
    private final Map<String, EMAComponentInstanceSymbol> library;

    ComponentInstanceSymbolLibrary() {
        library = new HashMap<>();
    }

    void addInstanceToLibrary(final String instanceIdentifier,
                              final String pathToModel,
                              final String rootModel) {
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabandTaggingResolver(pathToModel);
        EMAComponentInstanceSymbol emaComponentInstanceSymbol = taggingResolver
                .<EMAComponentInstanceSymbol>resolve(rootModel, EMAComponentInstanceSymbol.KIND)
                .orElseThrow(IllegalStateException::new);
        this.library.put(instanceIdentifier, emaComponentInstanceSymbol);
    }

    public EMAComponentInstanceSymbol getModelByIdentifier(final String instanceIdentifier) {
        return this.library.get(instanceIdentifier);
    }
}
