/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._symboltable;

import com.google.common.collect.Lists;
import de.monticore.lang.monticar.cnntrain.annotations.PreprocessingComponentParameter;
import de.monticore.lang.monticar.cnntrain.annotations.PreprocessingComponentParameter;
import de.monticore.symboltable.CommonSymbol;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 *
 */
public class PreprocessingComponentSymbol extends CommonSymbol {
    public static final PreprocessingComponentSymbolKind KIND = new PreprocessingComponentSymbolKind();

    private List<String> preprocessingComponentName;
    private PreprocessingComponentParameter preprocessingComponentParameter;

    public PreprocessingComponentSymbol(String name) {
        super(name, KIND);
        preprocessingComponentName = new ArrayList<>();
    }

    protected void setPreprocessingComponentName(List<String> preprocessingComponentNamePath) {
        this.preprocessingComponentName = Lists.newArrayList(preprocessingComponentNamePath);
    }

    public List<String> getPreprocessingComponentName() {
        return Lists.newArrayList(preprocessingComponentName);
    }

    public void setPreprocessingComponentParameter(PreprocessingComponentParameter preprocessingComponentParameter) {
        this.preprocessingComponentParameter = preprocessingComponentParameter;
    }

    public Optional<PreprocessingComponentParameter> getPreprocessingComponentParameter() {
        return Optional.ofNullable(preprocessingComponentParameter);
    }
}
