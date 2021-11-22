package schemalang._cocos;

import com.google.common.collect.Lists;
import de.se_rwth.commons.logging.Log;
import schemalang._ast.ASTSchemaDefinition;
import schemalang._ast.ASTTypedDeclaration;
import schemalang._ast.ASTRequiresRule;

import java.util.List;
import java.util.Optional;

import static schemalang.ErrorCodes.*;

/**
 * Context condition to check whether the members of a schema definition are unique.
 */
public class RequiresRulesReferenceValidMembers implements SchemaLangASTSchemaDefinitionCoCo {

    @Override
    public void check(ASTSchemaDefinition schema) {
        List<ASTRequiresRule> allRequiresRules = Lists.newArrayList();
        List<ASTTypedDeclaration> allBasicSchemaProperties = Lists.newArrayList();
        for (ASTSchemaDefinition superSchemaDefinition : schema.getSuperSchemaDefinitions()) {
            allRequiresRules.addAll(superSchemaDefinition.getRequiresRuleDefinitions());
            allBasicSchemaProperties.addAll(superSchemaDefinition.getBasicSchemaProperties());
        }
        allRequiresRules.addAll(schema.getRequiresRuleDefinitions());
        allBasicSchemaProperties.addAll(schema.getBasicSchemaProperties());

        for (ASTRequiresRule rule : allRequiresRules) {
            Optional<ASTTypedDeclaration> property = allBasicSchemaProperties.stream().filter(p -> rule.getName()
                    .equals(p.getName())).findFirst();
            if (!property.isPresent()) {
                Log.error(ERROR_CODE_SL_20C.concat(String.format(ERROR_MSG_SL_20C, rule.getName(), rule)),
                        rule.get_SourcePositionStart());
            }

            for (String member : rule.getDependenciesList()) {
                property = allBasicSchemaProperties.stream().filter(p -> member.equals(p.getName())).findFirst();
                if (!property.isPresent()) {
                    Log.error(ERROR_CODE_SL_20C.concat(String.format(ERROR_MSG_SL_20C, member, rule)),
                            rule.get_SourcePositionStart());
                }
            }
        }
    }
}