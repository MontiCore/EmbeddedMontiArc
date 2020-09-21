/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable;

import de.monticore.CommonModelingLanguage;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcModelLoader;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableCreator;
import de.monticore.symboltable.*;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;
import java.util.Optional;

public class EmbeddedMontiArcDynamicModelLoader extends EmbeddedMontiArcDynamicModelLoaderTOP {

    public EmbeddedMontiArcDynamicModelLoader(EmbeddedMontiArcDynamicLanguage language) {
        super(language);
    }


    @Override
    protected void createSymbolTableFromAST(ASTEMACompilationUnit ast, String modelName, MutableScope enclosingScope, ResolvingConfiguration resolvingConfiguration) {



        EmbeddedMontiArcDynamicSymbolTableCreator symbolTableCreator = (EmbeddedMontiArcDynamicSymbolTableCreator) this.getModelingLanguage().getSymbolTableCreator(resolvingConfiguration, enclosingScope).orElse(null);
        if (symbolTableCreator != null) {
            Log.debug("Start creation of symbol table for model \"" + modelName + "\".", EmbeddedMontiArcModelLoader.class.getSimpleName());
            Scope scope = symbolTableCreator.createFromAST(ast);
            if (!(scope instanceof ArtifactScope)) {
                Log.warn("0xA7001x392 Top scope of model " + modelName + " is expected to be an artifact scope, but is scope \"" + scope.getName() + "\"");
            }

            Log.debug("Created symbol table for model \"" + modelName + "\".", EmbeddedMontiArcModelLoader.class.getSimpleName());
        } else {
            Log.warn("0xA7002x392 No symbol created, because '" + this.getModelingLanguage().getName() + "' does not define a symbol table creator.");
        }


    }
}
