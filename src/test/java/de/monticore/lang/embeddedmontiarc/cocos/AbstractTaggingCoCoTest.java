/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.cocos;

import de.monticore.ast.ASTNode;
import de.monticore.lang.embeddedmontiarc.AbstractTaggingResolverTest;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcCoCoChecker;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class AbstractTaggingCoCoTest extends AbstractTaggingResolverTest {

    private void resolveTags(TaggingResolver taggingResolver, EMAComponentSymbol emaComponentSymbol){
        taggingResolver.getTags(emaComponentSymbol, RosConnectionSymbol.KIND);
        emaComponentSymbol.getSubComponents().forEach(sub -> resolveTags(taggingResolver,sub.getComponentType().getReferencedSymbol()));
    }

    public void testCoCosOnComponent(String componentName, String... expectedErrors) {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");

        EMAComponentSymbol component = taggingResolver.<EMAComponentSymbol>resolve(componentName, EMAComponentSymbol.KIND).orElse(null);
        assertNotNull(component);

        resolveTags(taggingResolver,component);

        ASTNode tmpNode = component.getAstNode().orElse(null);
        assertNotNull(tmpNode);
        ASTComponent astComponent = tmpNode instanceof ASTComponent ? (ASTComponent) tmpNode : null;
        assertNotNull(astComponent);

        EmbeddedMontiArcCoCoChecker checker = EmbeddedMontiArcCoCos.createChecker();
        checker.checkAll(astComponent);

        List<String> findings = Log.getFindings().stream().map(Finding::getMsg).collect(Collectors.toList());
        Arrays.stream(expectedErrors)
                .forEach(e -> {
                    boolean found = false;
                    for(String f : findings){
                        if(f.contains(e)){
                            found = true;
                            break;
                        }
                    }
                    assertTrue(found);
                });
    }
}
