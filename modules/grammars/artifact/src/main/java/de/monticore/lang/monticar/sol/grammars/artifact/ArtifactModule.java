/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.artifact;

import com.google.inject.AbstractModule;
import com.google.inject.Provides;
import com.google.inject.multibindings.Multibinder;
import de.monticore.lang.monticar.sol.grammars.artifact._cocos.ArtifactCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.artifact.cocos.*;
import de.monticore.lang.monticar.sol.runtime.grammar.cocos.CoCo;
import de.monticore.lang.monticar.sol.runtime.grammar.cocos.DuplicateCoCoErrorCode;

import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

public class ArtifactModule extends AbstractModule {
    @Override
    protected void configure() {
        this.addBindings();
        this.addMultiBindings();
    }

    private void addBindings() { /* Placeholder */ }

    private void addMultiBindings() {
        this.addContextConditions();
    }

    private void addContextConditions() {
        Multibinder<ArtifactCoCo> coCos = Multibinder.newSetBinder(binder(), ArtifactCoCo.class);

        coCos.addBinding().to(RequiredAttributesCoCo.class);
        coCos.addBinding().to(ArtifactNameCoCo.class);
        coCos.addBinding().to(UniqueAttributeCoCo.class);
        coCos.addBinding().to(UniqueArtifactCoCo.class);
        coCos.addBinding().to(ExistingReferenceCoCo.class);
    }

    @Provides
    protected ArtifactCoCoChecker provideCoCoChecker(Set<ArtifactCoCo> coCos) throws DuplicateCoCoErrorCode {
        ArtifactCoCoChecker checker = new ArtifactCoCoChecker();
        Set<String> errorCodes = coCos.stream().map(CoCo::getErrorCode).collect(Collectors.toSet());
        List<String> workingList = coCos.stream().map(CoCo::getErrorCode).collect(Collectors.toList());

        errorCodes.forEach(workingList::remove);

        if (workingList.isEmpty()) coCos.forEach(coCo -> coCo.registerTo(checker));
        else throw new DuplicateCoCoErrorCode(workingList);

        return checker;
    }
}
