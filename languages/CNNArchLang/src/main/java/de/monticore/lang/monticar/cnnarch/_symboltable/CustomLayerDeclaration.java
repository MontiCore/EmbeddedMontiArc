/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnnarch._symboltable;

import java.io.File;
import java.util.*;

public abstract class CustomLayerDeclaration extends LayerDeclarationSymbol implements LayerComputeOutputTypes{

    private File customFilePath;
    private String language;

    public CustomLayerDeclaration(String name, File customFilePath, String language) {
        super(name);
        setCustomFilePath(customFilePath);
        setLanguage(language);
    }

    public void setParameters(List<ParameterSymbol> parameters) {
        super.setParameters(parameters);
        for (ParameterSymbol param : parameters) {
            param.putInScope(getSpannedScope());
        }
    }

    private void setCustomFilePath(File customFilePath){
        this.customFilePath = customFilePath;
    }

    public File getCustomFilePath(){
        return this.customFilePath;
    }

    public String getLanguage(){
        return this.language;
    }

    private void setLanguage(String language){
        this.language = language;
    }

    public abstract List<ParameterSymbol> extractParametersFromFile();

    public abstract List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member);

    public abstract CustomLayerDeclaration deepCopy();

    @Override
    public boolean isCustom(){
        return true;
    }

}
