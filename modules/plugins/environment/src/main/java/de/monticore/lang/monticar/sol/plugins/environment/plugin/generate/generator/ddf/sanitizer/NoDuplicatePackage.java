/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.generator.ddf.sanitizer;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstallInstruction;
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
public class NoDuplicatePackage implements DDFSanitizerPhase {
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
        Predicate<ASTInstallInstruction> isEmpty = node -> node.sizePackages() == 0;
        Function<ASTInstruction, ASTInstallInstruction> cast = instruction -> (ASTInstallInstruction)instruction;
        Stream<ASTInstallInstruction> stream = instructions.stream().filter(instanceOf).map(cast);
        List<ASTInstruction> duplicates = stream.map(this::removeDuplicates).filter(isEmpty).collect(Collectors.toList());

        instructions.removeAll(duplicates);
    }

    protected ASTInstallInstruction removeDuplicates(ASTInstallInstruction instruction) {
        Predicate<ASTStringLiteral> notContainedIn = node -> !this.installations.contains(node.getValue());
        Stream<ASTStringLiteral> packageStream = instruction.streamPackages().filter(notContainedIn);
        List<ASTStringLiteral> packageList = packageStream.collect(Collectors.toList());
        Consumer<ASTStringLiteral> add = node -> this.installations.add(node.getValue());

        instruction.setPackageList(packageList);
        packageList.forEach(add);

        return instruction;
    }
}
