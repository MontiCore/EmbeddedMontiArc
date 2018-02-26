package de.monticore.lang.monticar.generator.middleware.impls;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public interface GeneratorImpl {
    default List<File> generate(ExpandedComponentInstanceSymbol componentInstanceSymbol, TaggingResolver taggingResolver) throws IOException {
        return new ArrayList<>();
    }

    default void setGenerationTargetPath(String path) {

    }

    default boolean willAccept(ExpandedComponentInstanceSymbol componentInstanceSymbol) {
        return true;
    }
}
