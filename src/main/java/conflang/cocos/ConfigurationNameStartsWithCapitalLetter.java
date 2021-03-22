/* (c) https://github.com/MontiCore/monticore */
package conflang.cocos;

import conflang._ast.ASTConfLang;
import conflang._cocos.ConfLangASTConfLangCoCo;
import de.se_rwth.commons.logging.Log;

public class ConfigurationNameStartsWithCapitalLetter implements ConfLangASTConfLangCoCo {

    @Override
    public void check(ASTConfLang configuration) {
        String configurationName = configuration.getName();
        boolean startsWithUpperCase = Character.isUpperCase(configurationName.charAt(0));

        if (!startsWithUpperCase) {
            Log.error(String.format("0xB4002 Configuration name '%s' must start with a capital letter.", configurationName),
                    configuration.get_SourcePositionStart());
        }
    }
}
