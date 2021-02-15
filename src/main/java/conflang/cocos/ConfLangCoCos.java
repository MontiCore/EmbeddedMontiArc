/* (c) https://github.com/MontiCore/monticore */
package conflang.cocos;

import conflang._cocos.ConfLangCoCoChecker;

public class ConfLangCoCos {

    public ConfLangCoCoChecker getCheckerForAllCoCos() {
        final ConfLangCoCoChecker checker = new ConfLangCoCoChecker();
        checker.addCoCo(new ConfigurationNameStartsWithCapitalLetter());
        checker.addCoCo(new ConfigurationEntriesAreUnique());
        return checker;
    }
}
