package schemalang._cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.se_rwth.commons.logging.Log;
import schemalang._ast.ASTSchemaDefinition;
import schemalang._symboltable.ReferenceModel;
import schemalang._symboltable.SchemaDefinitionSymbol;
import schemalang._symboltable.TypedDeclarationSymbol;

import java.util.Collection;
import java.util.Optional;

import static schemalang.ErrorCodes.ERROR_CODE_TA_17C;
import static schemalang.ErrorCodes.ERROR_MSG_TA_17C;

public class ComponentsDoNotConflictWithReferenceModel implements SchemaLangASTSchemaDefinitionCoCo {

    @Override
    public void check(ASTSchemaDefinition definition) {

        SchemaDefinitionSymbol schemaDefinitionSymbol = definition.getSchemaDefinitionSymbol();
        Collection<ReferenceModel> referenceModels = schemaDefinitionSymbol.getReferenceModels();
        if (referenceModels == null || referenceModels.isEmpty()) {
            return;
        }

        for (ReferenceModel referenceModel : referenceModels) {
            checkForConflict(definition, schemaDefinitionSymbol, referenceModel.getEMAComponent());
        }
    }

    private void checkForConflict(ASTSchemaDefinition definition,
                                  SchemaDefinitionSymbol schemaDefinitionSymbol,
                                  EMAComponentSymbol emaComponentSymbol) {

        Collection<EMAComponentInstantiationSymbol> subComponents = emaComponentSymbol.getSubComponents();
        for (EMAComponentInstantiationSymbol subComponent : subComponents) {
            Optional<TypedDeclarationSymbol> typedDeclaration = schemaDefinitionSymbol.getTypedDeclaration(subComponent.getName());
            if (typedDeclaration.isPresent()) {
                Log.error(ERROR_CODE_TA_17C.concat(String.format(ERROR_MSG_TA_17C, subComponent.getName(),
                        definition.getName(), emaComponentSymbol.getFullName())),
                        definition.getReferenceModel().get_SourcePositionStart());
            }
        }
    }
}