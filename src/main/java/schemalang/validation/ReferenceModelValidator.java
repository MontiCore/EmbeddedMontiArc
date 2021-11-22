package schemalang.validation;

import com.google.common.collect.Lists;
import conflang._symboltable.ConfigurationSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import schemalang._symboltable.SchemaDefinitionSymbol;
import schemalang.validation.cocos.ReferenceModelIsValid;
import schemalang.validation.model.ArchitectureComponent;

import java.util.Collection;
import java.util.List;
import java.util.Map;

import static schemalang.validation.ValidationHelpers.getReferenceModels;

public class ReferenceModelValidator {

    public static List<ReferenceModelViolation> validate(SchemaDefinitionSymbol schema,
                                                         ConfigurationSymbol configuration,
                                                         Map<String, ArchitectureComponent> componentMap) {

        return validate(Lists.newArrayList(schema), configuration, componentMap);
    }

    public static List<ReferenceModelViolation> validate(List<SchemaDefinitionSymbol> schemas,
                                                 ConfigurationSymbol configuration,
                                                 Map<String, ArchitectureComponent> componentMap) {

        Collection<EMAComponentSymbol> referenceModels = getReferenceModels(schemas);
        if (!referenceModels.isEmpty()) {
            List<ReferenceModelViolation> allViolations = Lists.newArrayList();
            for (EMAComponentSymbol referenceModel : referenceModels) {
                ReferenceModelViolation violation = ReferenceModelIsValid.validateReferenceModel(componentMap,
                        referenceModel, configuration, false);

                // If one of the reference models succeeds, we stop the validation
                if (violation.getViolations().isEmpty()) {
                    return Lists.newArrayList();
                }
                allViolations.add(violation);
            }
            return allViolations;
        }
        return Lists.newArrayList();
    }
}