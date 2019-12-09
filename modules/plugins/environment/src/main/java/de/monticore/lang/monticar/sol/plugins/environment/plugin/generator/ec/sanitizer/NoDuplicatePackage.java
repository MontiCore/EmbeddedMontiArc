/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.sanitizer;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstall;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstruction;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.mcliterals._ast.ASTStringLiteral;

import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.stream.Collectors;
import java.util.stream.Stream;

@Singleton
public class NoDuplicatePackage implements ECSanitizerPhase {
    protected final NotificationService notifications;

    protected Set<String> installations;

    @Inject
    protected NoDuplicatePackage(NotificationService notifications) {
        this.notifications = notifications;
        this.installations = new HashSet<>();
    }

    @Override
    public void sanitize(List<ASTInstruction> instructions) {
        this.notifications.info("Removing Duplicate Installations.");
        this.installations.clear();

        Predicate<ASTInstruction> instanceOf = instruction -> instruction.getType().equals("INSTALL");
        Predicate<ASTInstall> isEmpty = node -> node.sizePackages() == 0;
        Function<ASTInstruction, ASTInstall> cast = instruction -> (ASTInstall)instruction;
        Stream<ASTInstall> stream = instructions.stream().filter(instanceOf).map(cast);
        List<ASTInstruction> duplicates = stream.map(this::removeDuplicates).filter(isEmpty).collect(Collectors.toList());

        instructions.removeAll(duplicates);
    }

    protected ASTInstall removeDuplicates(ASTInstall instruction) {
        Predicate<ASTStringLiteral> notContainedIn = node -> !this.installations.contains(node.getValue());
        Stream<ASTStringLiteral> packageStream = instruction.streamPackages().filter(notContainedIn);
        List<ASTStringLiteral> packageList = packageStream.collect(Collectors.toList());
        Consumer<ASTStringLiteral> add = node -> this.installations.add(node.getValue());

        instruction.setPackageList(packageList);
        packageList.forEach(add);

        return instruction;
    }
}
