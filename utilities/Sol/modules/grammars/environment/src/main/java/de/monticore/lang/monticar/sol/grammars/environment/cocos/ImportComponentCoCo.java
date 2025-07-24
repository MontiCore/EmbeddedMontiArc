/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTDockerfile;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentASTDockerfileCoCo;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.DockerfileSymbol;

/**
 * This context condition checks whether a Non-Component Dockerfile is imported.
 */
public class ImportComponentCoCo extends CommonEnvironmentCoCo implements EnvironmentASTDockerfileCoCo {
    public ImportComponentCoCo() {
        super("ENV0008", "'%s' is not a Component Dockerfile and should therefore not be imported.");
    }

    @Override
    public void registerTo(EnvironmentCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTDockerfile node) {
        node.getDockerfileSymbolOpt().ifPresent(this::check);
    }

    protected void check(DockerfileSymbol symbol) {
        symbol.getComponentSymbols().forEach(component -> {
            if (!component.isComponent()) this.error(symbol, symbol.getName());
        });
    }
}
