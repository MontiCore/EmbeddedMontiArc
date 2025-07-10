package schemalang._cocos;

import com.google.common.collect.Lists;
import de.se_rwth.commons.logging.Log;
import schemalang.SchemaMemberType;
import schemalang._ast.ASTSchemaDefinition;
import schemalang._ast.ASTSchemaMember;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static schemalang.ErrorCodes.ERROR_CODE_SL_03C;
import static schemalang.ErrorCodes.ERROR_MSG_SL_03C;

/**
 * Context condition to check whether the members of a schema definition are unique.
 */
public class PropertyDefinitionsAreUnique implements SchemaLangASTSchemaDefinitionCoCo {

    @Override
    public void check(ASTSchemaDefinition definition) {
        List<ASTSchemaMember> allSchemaMembers = Lists.newArrayList();

        for (ASTSchemaDefinition superSchemaDefinition : definition.getSuperSchemaDefinitions()) {
            allSchemaMembers.addAll(superSchemaDefinition.getAllDeclarations());
        }

        allSchemaMembers.addAll(definition.getAllDeclarations());
        Map<String, Integer> resultMap = new HashMap<>();
        for (ASTSchemaMember member : allSchemaMembers) {
            if (resultMap.containsKey(member.getName()) && !SchemaMemberType.COMPLEX.equals(member.getSchemaMemberType())) {
                Log.error(ERROR_CODE_SL_03C.concat(String.format(ERROR_MSG_SL_03C, member.getName(), definition.getName())),
                        member.get_SourcePositionStart());
            } else {
                resultMap.put(member.getName(), 1);
            }
        }
    }
}