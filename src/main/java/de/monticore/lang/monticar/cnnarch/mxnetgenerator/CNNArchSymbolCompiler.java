package de.monticore.lang.monticar.cnnarch.mxnetgenerator;

import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.cnnarch._cocos.CNNArchCocos;
import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch.mxnetgenerator.checker.LayerSupportChecker;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.nio.file.Path;
import java.util.List;
import java.util.Optional;

public class CNNArchSymbolCompiler {
    private final LayerSupportChecker layerChecker;

    public CNNArchSymbolCompiler(final LayerSupportChecker layerChecker) {
        this.layerChecker = layerChecker;
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
        if (!supportCheck(compilationUnit.get().getArchitecture())){
            failWithMessage("Architecture not supported by generator");
        }

        return compilationUnit.get().getArchitecture();
    }

    private void failWithMessage(final String message) {
        Log.error(message);
        System.exit(1);
    }

    private boolean supportCheck(ArchitectureSymbol architecture){
        for (ArchitectureElementSymbol element : ((CompositeElementSymbol)architecture.getBody()).getElements()){
            if(!isSupportedLayer(element, layerChecker)) {
                return false;
            }
        }
        return true;
    }

    private boolean isSupportedLayer(ArchitectureElementSymbol element, LayerSupportChecker layerChecker){
        List<ArchitectureElementSymbol> constructLayerElemList;

        if (element.getResolvedThis().get() instanceof CompositeElementSymbol) {
            constructLayerElemList = ((CompositeElementSymbol)element.getResolvedThis().get()).getElements();
            for (ArchitectureElementSymbol constructedLayerElement : constructLayerElemList) {
                if (!isSupportedLayer(constructedLayerElement, layerChecker)) {
                    return false;
                }
            }
        }
        if (!layerChecker.isSupported(element.toString())) {
            Log.error("Unsupported layer " + "'" + element.getName() + "'" + " for the backend.");
            return false;
        } else {
            return true;
        }
    }
}
