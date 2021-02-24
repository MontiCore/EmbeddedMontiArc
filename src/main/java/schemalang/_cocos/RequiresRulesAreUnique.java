package schemalang._cocos;

import com.google.common.collect.Lists;
import de.se_rwth.commons.logging.Log;
import schemalang._ast.ASTRequiresRule;
import schemalang._ast.ASTSchemaDefinition;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static schemalang.ErrorCodes.*;

/**
 * Context condition to check whether the members of a schema definition are unique.
 */
public class RequiresRulesAreUnique implements SchemaLangASTSchemaDefinitionCoCo {

    @Override
    public void check(ASTSchemaDefinition definition) {
        List<ASTRequiresRule> allRequiresRules = Lists.newArrayList();
        for (ASTSchemaDefinition superSchemaDefinition : definition.getSuperSchemaDefinitions()) {
            allRequiresRules.addAll(superSchemaDefinition.getRequiresRuleDefinitions());
        }

        allRequiresRules.addAll(definition.getRequiresRuleDefinitions());
        Map<String, Integer> resultMap = new HashMap<>();
        for (ASTRequiresRule rule : allRequiresRules) {
            if (resultMap.containsKey(rule.getName())) {
                Log.error(ERROR_CODE_SL_19C.concat(String.format(ERROR_MSG_SL_19C, rule, definition.getName())),
                        rule.get_SourcePositionStart());
            } else {
                resultMap.put(rule.getName(), 1);
            }
        }
    }
}