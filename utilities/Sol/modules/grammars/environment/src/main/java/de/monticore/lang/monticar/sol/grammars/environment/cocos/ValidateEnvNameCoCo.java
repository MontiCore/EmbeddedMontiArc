/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTEnv;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentASTEnvCoCo;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentCoCoChecker;
import de.monticore.mcliterals._ast.ASTStringLiteral;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * This context condition checks whether an environmental variable has the right format.
 */
public class ValidateEnvNameCoCo extends CommonEnvironmentCoCo implements EnvironmentASTEnvCoCo {
    public ValidateEnvNameCoCo() {
        super("ENV0004", "Environmental Variable '%s' has not the right name format.");
    }

    @Override
    public void registerTo(EnvironmentCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTEnv node) {
        // https://stackoverflow.com/questions/2821043/allowed-characters-in-linux-environment-variable-names
        ASTStringLiteral envNode = node.getKey();
        String env = envNode.getValue();
        Matcher matcher = Pattern.compile("[[A-Z]_][[A-Z]_[0-9]]*").matcher(env);

        if (!matcher.matches()) this.warn(node, env);
    }
}
