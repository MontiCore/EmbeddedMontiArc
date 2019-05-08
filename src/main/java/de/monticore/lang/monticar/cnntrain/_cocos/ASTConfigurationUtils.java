package de.monticore.lang.monticar.cnntrain._cocos;

import de.monticore.lang.monticar.cnntrain._ast.ASTConfiguration;
import de.monticore.lang.monticar.cnntrain._ast.ASTEnvironmentEntry;
import de.monticore.lang.monticar.cnntrain._ast.ASTLearningMethodEntry;

class ASTConfigurationUtils {
    static boolean isReinforcementLearning(final ASTConfiguration configuration) {
        return configuration.getEntriesList().stream().anyMatch(e ->
                (e instanceof ASTLearningMethodEntry)
                        && ((ASTLearningMethodEntry)e).getValue().isPresentReinforcement());
    }

    static boolean hasEnvironment(final ASTConfiguration configuration) {
        return configuration.getEntriesList().stream().anyMatch(e -> e instanceof ASTEnvironmentEntry);
    }
}
