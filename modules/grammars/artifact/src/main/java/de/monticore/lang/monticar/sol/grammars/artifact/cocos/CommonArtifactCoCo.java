/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.artifact.cocos;

import de.monticore.lang.monticar.sol.grammars.artifact._cocos.ArtifactCoCoChecker;
import de.monticore.lang.monticar.sol.runtime.grammar.cocos.CommonCoCo;

public abstract class CommonArtifactCoCo extends CommonCoCo<ArtifactCoCoChecker> implements ArtifactCoCo {
    protected CommonArtifactCoCo(String errorCode, String schema) {
        super(errorCode, schema);
    }
}
