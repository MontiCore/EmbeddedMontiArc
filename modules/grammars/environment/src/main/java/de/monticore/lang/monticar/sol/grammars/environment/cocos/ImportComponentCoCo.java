/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTDockerfile;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentASTDockerfileCoCo;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentCoCoChecker;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * This context condition checks whether a Non-Component Dockerfile is imported.
 */
public class ImportComponentCoCo implements EnvironmentASTDockerfileCoCo, EnvironmentCoCo {
    @Override
    public String getErrorCode() {
        return "ENV0008";
    }

    @Override
    public String getErrorMessage(Object... parameters) {
        List<Object> parameterList = new ArrayList<>(Arrays.asList(parameters));

        parameterList.add(0, this.getErrorCode());
        return String.format("%s '%s' is not a Component Dockerfile and should therefore not be imported.", parameterList.toArray());
    }

    @Override
    public void registerTo(EnvironmentCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTDockerfile node) {
        node.getDockerfileSymbolOpt().ifPresent(symbol -> {
            symbol.getComponentSymbols().forEach(component -> {
                if (!component.isComponent()) {
                    ASTDockerfile componentNode = component.getDockerfileNode().orElse(node);

                    Log.warn(this.getErrorMessage(component.getFullName()), componentNode.get_SourcePositionStart());
                }
            });
        });
    }
}
