/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.options.cocos;

import com.google.inject.Inject;
import de.monticore.lang.monticar.sol.grammars.options._ast.ASTOption;
import de.monticore.lang.monticar.sol.grammars.options._cocos.OptionsASTOptionCoCo;
import de.monticore.lang.monticar.sol.grammars.options._cocos.OptionsCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.options.cocos.types.OptionType;
import de.se_rwth.commons.logging.Log;

import java.util.*;

/**
 * This context condition checks whether the type of a given option is supported.
 */
public class SupportedTypeCoCo implements OptionCoCo, OptionsASTOptionCoCo {
    protected final Map<String, OptionType> types;

    @Inject
    protected SupportedTypeCoCo(Map<String, OptionType> types) {
        this.types = types;
    }

    @Override
    public String getErrorCode() {
        return "OPT0000";
    }

    @Override
    public String getErrorMessage(Object... parameters) {
        List<Object> parameterList = new ArrayList<>(Arrays.asList(parameters));

        parameterList.add(0, this.getErrorCode());
        return String.format("%s Use of unregistered option type '%s'.", parameterList.toArray());
    }

    @Override
    public void registerTo(OptionsCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTOption node) {
        String type = node.getType();
        Set<String> types = this.types.keySet();

        if (!types.contains(type)) Log.warn(this.getErrorMessage(type), node.get_SourcePositionStart());
    }
}
