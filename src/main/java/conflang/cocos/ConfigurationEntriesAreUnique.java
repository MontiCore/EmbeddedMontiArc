/* (c) https://github.com/MontiCore/monticore */
package conflang.cocos;

import com.google.common.collect.Sets;
import conflang._ast.ASTConfLang;
import conflang._ast.ASTConfigurationEntry;
import conflang._cocos.ConfLangASTConfLangCoCo;
import de.se_rwth.commons.logging.Log;

import java.util.List;
import java.util.Set;

public class ConfigurationEntriesAreUnique implements ConfLangASTConfLangCoCo {

    @Override
    public void check(ASTConfLang configuration) {
        List<ASTConfigurationEntry> configurationEntries = configuration.getConfigurationEntryList();
        Set<String> configurationNames = Sets.newHashSet();

        for (ASTConfigurationEntry entry : configurationEntries) {
            configurationNames.add(entry.getName());
        }

        boolean areEntriesUnique = configurationEntries.size() == configurationNames.size();
        if (!areEntriesUnique) {
            Log.error("0xB4002 Configuration entries must not be repeated.",
                    configuration.get_SourcePositionStart());
        }
    }
}
