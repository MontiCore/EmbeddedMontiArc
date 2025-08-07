/* (c) https://github.com/MontiCore/monticore */
package conflang._cocos;

import conflang._ast.ASTConfiguration;
import de.se_rwth.commons.logging.Log;

import static conflang.ErrorCodes.*;

public class ConfigurationNameStartsWithCapitalLetter implements ConfLangASTConfigurationCoCo {

    @Override
    public void check(ASTConfiguration configuration) {
        String configurationName = configuration.getName();
        boolean startsWithUpperCase = Character.isUpperCase(configurationName.charAt(0));

        if (!startsWithUpperCase) {
            Log.error(ERROR_CODE_CL_02C.concat(String.format(ERROR_MSG_CL_02C, configurationName)),
                    configuration.get_SourcePositionStart());
        }
    }
}