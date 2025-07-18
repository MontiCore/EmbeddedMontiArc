package conflang._cocos;

public class ConfLangCocoFactory {

    public static ConfLangCoCoChecker createCheckerWithMinimumCoCos() {
        final ConfLangCoCoChecker checker = new ConfLangCoCoChecker();
        checker.addCoCo(new ConfigurationNameStartsWithCapitalLetter());
        checker.addCoCo(new ConfigurationEntriesAreUnique());
        return checker;
    }

    public static ConfLangCoCoChecker createCheckerWithAllCoCos() {
        final ConfLangCoCoChecker checker = new ConfLangCoCoChecker();
        checker.addCoCo(new ConfigurationNameStartsWithCapitalLetter());
        checker.addCoCo(new ConfigurationEntriesAreUnique());
        checker.addCoCo(new ConfigurationEntriesInNestedEntryAreUnique());
        return checker;
    }

    public static ConfLangCoCoChecker createCheckerWithCoCo(ConfLangASTConfigurationCoCo coco) {
        final ConfLangCoCoChecker checker = new ConfLangCoCoChecker();
        checker.addCoCo(coco);
        return checker;
    }

    public static ConfLangCoCoChecker createCheckerWithCoCo(ConfLangASTNestedConfigurationEntryCoCo coco) {
        final ConfLangCoCoChecker checker = new ConfLangCoCoChecker();
        checker.addCoCo(coco);
        return checker;
    }
}