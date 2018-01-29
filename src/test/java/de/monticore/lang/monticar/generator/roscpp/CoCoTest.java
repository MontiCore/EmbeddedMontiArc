package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._cocos.EmbeddedMontiArcMathCoCoChecker;
import de.monticore.lang.monticar.generator.roscpp.cocos.EmbeddedMontiArcMathRosCppCoCos;
import de.monticore.lang.monticar.generator.roscpp.tagging.RosConnectionSymbol;
import de.monticore.lang.monticar.generator.roscpp.tagging.RosToEmamTagSchema;
import de.monticore.lang.tagging._symboltable.TagSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.util.Collection;

import static org.junit.Assert.assertNotNull;

public class CoCoTest extends AbstractSymtabTest {

    @Test
    public void testRosCppCoCos() {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        EmbeddedMontiArcMathCoCoChecker checker = EmbeddedMontiArcMathRosCppCoCos.createChecker();


        ExpandedComponentInstanceSymbol component = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.cocos.rosToRosComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        ExpandedComponentInstanceSymbol subComp1 = component.getSubComponent("subComp1").orElse(null);
        ExpandedComponentInstanceSymbol subComp2 = component.getSubComponent("subComp2").orElse(null);

//        ComponentInstanceSymbol component = taggingResolver.<ComponentInstanceSymbol>resolve("tests.cocos.rosToRosComp", ComponentInstanceSymbol.KIND).orElse(null);
//        ComponentInstanceSymbol subComp1 = component.getSubComponent("subComp1").orElse(null);
//        ComponentInstanceSymbol subComp2 = component.getSubComponent("subComp2").orElse(null);

        assertNotNull(subComp1);
        assertNotNull(subComp2);

        Collection<TagSymbol> tags1 = taggingResolver.getTags(subComp1.getPort("outPort").orElse(null), RosConnectionSymbol.KIND);
        Collection<TagSymbol> tags2 = taggingResolver.getTags(subComp2.getPort("inPort").orElse(null), RosConnectionSymbol.KIND);


        ASTComponent astNode = (ASTComponent) component.getAstNode().orElse(null);
        assertNotNull(astNode);

        checker.checkAll(astNode);

    }
}
