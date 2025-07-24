/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.sanitizer;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstruction;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;

import java.util.List;
import java.util.Set;

@Singleton
public class ECSanitizerImpl implements ECSanitizer {
    protected final NotificationService notifications;
    protected final Set<ECSanitizerPhase> contributions;

    @Inject
    protected ECSanitizerImpl(NotificationService notifications, Set<ECSanitizerPhase> contributions) {
        this.notifications = notifications;
        this.contributions = contributions;
    }

    @Override
    public List<ASTInstruction> sanitize(List<ASTInstruction> instructions) {
        this.notifications.info("Sanitizing Instructions.");

        for(ECSanitizerPhase contribution : this.contributions) {
            contribution.sanitize(instructions);
        }

        return instructions;
    }
}
