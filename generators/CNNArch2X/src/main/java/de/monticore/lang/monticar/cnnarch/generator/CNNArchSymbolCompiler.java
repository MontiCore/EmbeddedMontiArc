/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.generator;

import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.cnnarch._cocos.CNNArchCocos;
import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.nio.file.Path;
import java.util.List;
import java.util.Optional;

public class CNNArchSymbolCompiler {
    private final ArchitectureSupportChecker architectureSupportChecker;
    private final LayerSupportChecker layerSupportChecker;

    public CNNArchSymbolCompiler(final ArchitectureSupportChecker architectureSupportChecker,
                                 final LayerSupportChecker layerSupportChecker) {
        this.architectureSupportChecker = architectureSupportChecker;
        this.layerSupportChecker = layerSupportChecker;
    }

     public ArchitectureSymbol compileArchitectureSymbolFromModelsDir(
        final Path modelsDirPath, final String rootModel) {
        ModelPath mp = new ModelPath(modelsDirPath);
        GlobalScope scope = new GlobalScope(mp, new CNNArchLanguage());
        return compileArchitectureSymbol(scope, rootModel);
     }

    public ArchitectureSymbol compileArchitectureSymbol(Scope scope, String rootModelName) {
        Optional<CNNArchCompilationUnitSymbol> compilationUnit = scope.resolve(rootModelName, CNNArchCompilationUnitSymbol.KIND);
        if (!compilationUnit.isPresent()){
            failWithMessage("Could not resolve architecture " + rootModelName);
        }

        CNNArchCocos.checkAll(compilationUnit.get());

        ArchitectureSymbol architecture = compilationUnit.get().getArchitecture();

        if (!architectureSupportChecker.check(architecture) || !layerSupportChecker.check(architecture)) {
            failWithMessage("Architecture not supported by generator");
        }

        return architecture;
    }

    private void failWithMessage(final String message) {
        Log.error(message);
        System.exit(1);
    }
}
