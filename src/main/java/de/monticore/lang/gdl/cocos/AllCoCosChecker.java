package de.monticore.lang.gdl.cocos;

import de.monticore.lang.gdl._cocos.GDLCoCoChecker;

public class AllCoCosChecker extends GDLCoCoChecker {
    
    public AllCoCosChecker() {
        super();
        addCoCo(new InitOnlyOnRoot());
    }

}
