package de.monticore.lang.gdl.cocos.types;

import de.monticore.lang.gdl._cocos.GDLCoCoChecker;

public class TypesCoCosChecker extends GDLCoCoChecker {
    
    public TypesCoCosChecker() {
        super();
        addCoCo(new RootTypeDefNoTokens());
        addCoCo(new TypeMapDefNoTokens());
        addCoCo(new TypesBoundOnce());
        addCoCo(new StateActionSpacesTyped());
    }

}
