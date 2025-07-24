package schemalang._cocos;

public class SchemaLangCocoFactory {

    public static SchemaLangCoCoChecker getCheckerWithAllCoCos() {
        final SchemaLangCoCoChecker checker = new SchemaLangCoCoChecker();
        checker.addCoCo(new ComponentsDoNotConflictWithReferenceModel());
        checker.addCoCo(new CustomPropertyDefinitionExists());
        checker.addCoCo(new CustomPropertyDefinitionOverrideOnlyInSubSchema());
        checker.addCoCo(new CustomPropertyHasDifferentName());
        checker.addCoCo(new DomainValuesCompatibleWithType());
        checker.addCoCo(new FileAndSchemaNameAreEqual());
        checker.addCoCo(new DefaultValueCompatibleWithType());
        checker.addCoCo(new DefaultValueIsInDomain());
        checker.addCoCo(new PropertyDefinitionsAreUnique());
        checker.addCoCo(new RequiresRulesAreUnique());
        checker.addCoCo(new RequiresRulesReferenceValidMembers());
        checker.addCoCo(new SchemaCausesReferenceCycle());
        checker.addCoCo(new SchemaEnumsOnlyOneAllowed());
        checker.addCoCo(new SchemaLinksAreTypeless());
        checker.addCoCo(new SchemaNameMustStartWithCapitalLetter());
        checker.addCoCo(new RangeAndDomainDontCoOccur());
        return checker;
    }

    public static SchemaLangCoCoChecker createEmptyCoCoChecker() {
        return new SchemaLangCoCoChecker();
    }
}