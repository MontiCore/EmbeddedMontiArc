/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.printer;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.GeneratorEngine;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.mp.ModelPathService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage;
import de.monticore.symboltable.Symbol;

@Singleton
public class ImportPrinterImpl extends NamePrinterImpl implements ImportPrinter {
    protected final ModelPathService modelPath;

    @Inject
    protected ImportPrinterImpl(GeneratorEngine engine, ModelPathService modelPath) {
        super(engine);

        this.modelPath = modelPath;
    }

    @Override
    public String printImport(SolPackage rootPackage, Symbol symbol, String extension, String relativePath) {
        String qualifiedName = symbol.getFullName();
        String qualifiedPath = this.printQualifiedPath(symbol);
        String filledRelativePath = relativePath.contains("%s") ? String.format(relativePath, qualifiedPath) : relativePath;

        if (this.modelPath.isModelLocal(rootPackage, qualifiedName, extension)) return filledRelativePath;
        else return this.doPrintImport(rootPackage, qualifiedName, extension, filledRelativePath);
    }

    protected String doPrintImport(SolPackage rootPackage, String qualifiedName, String extension, String relativePath) {
        SolPackage ownerPackage = this.modelPath.locateModel(rootPackage, qualifiedName, extension)
                .orElseThrow(() -> new RuntimeException("Model could not be located."));
        String name = ownerPackage.getName()
                .orElseThrow(() -> new RuntimeException("Owner Package does not have a name."));
        String normalizedPath = relativePath.replace("../", "");

        return String.format("%s/lib/%s", name, normalizedPath);
    }
}
