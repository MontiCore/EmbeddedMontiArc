package de.monticore.lang.gdl.cocos;

import de.monticore.lang.gdl._cocos.GDLCoCoChecker;

public class AllCoCosChecker extends GDLCoCoChecker {
    
    public AllCoCosChecker() {
        super();
        addCoCo(new SeesOnlyInStateQuery());
        addCoCo(new KeywordsOnFirstPositionOnly());
        addCoCo(new MatchTupleArityMin());
        addCoCo(new MatchTupleArityExact());
        addCoCo(new KeywordOnlyInContext());
    }

}
