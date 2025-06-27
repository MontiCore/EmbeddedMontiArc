/* (c) https://github.com/MontiCore/monticore */
package conflang._cocos;

import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import conflang._ast.ASTConfiguration;
import conflang._ast.ASTConfigurationEntry;
import de.se_rwth.commons.logging.Log;

import java.util.List;
import java.util.Map;

import static conflang.ErrorCodes.ERROR_CODE_CL_01C;
import static conflang.ErrorCodes.ERROR_MSG_CL_01C;

public class ConfigurationEntriesAreUnique implements ConfLangASTConfigurationCoCo {

    @Override
    public void check(ASTConfiguration configuration) {
        List<ASTConfigurationEntry> configurationEntries = configuration.getAllConfigurationEntries();

        Map<String, List<ASTConfigurationEntry>> configurationNameCounts = Maps.newHashMap();
        for (ASTConfigurationEntry entry : configurationEntries) {
            if (!configurationNameCounts.containsKey(entry.getName())) {
                configurationNameCounts.put(entry.getName(), Lists.newArrayList());
            }
            List<ASTConfigurationEntry> entries = configurationNameCounts.get(entry.getName());
            entries.add(entry);
        }

        List<ASTConfigurationEntry> entries;
        for (Map.Entry<String, List<ASTConfigurationEntry>> configurationNameCount : configurationNameCounts.entrySet()) {
            entries = configurationNameCount.getValue();
            if (entries.size() > 1) {
                Log.error(ERROR_CODE_CL_01C.concat(String.format(ERROR_MSG_CL_01C, configurationNameCount.getKey())),
                        entries.get(1).get_SourcePositionStart());
            }
        }
    }
}