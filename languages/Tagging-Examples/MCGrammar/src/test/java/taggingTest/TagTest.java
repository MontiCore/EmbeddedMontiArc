/* (c) https://github.com/MontiCore/monticore */
package taggingTest;

import java.nio.file.Paths;
import java.util.*;

import de.monticore.ModelingLanguageFamily;
import de.monticore.grammar.symboltable.MCGrammarSymbol;
import de.monticore.grammar.symboltable.MCProdSymbol;
import de.monticore.grammar.symboltable.MontiCoreGrammarLanguage;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.tagging._symboltable.TagSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.GlobalScope;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;

/**
 * Created by Michael von Wenckstern on 30.05.2016.
 *
 */
public class TagTest {

    protected static MontiCoreGrammarLanguage getMontiCoreGrammarLanguage() {
        MontiCoreGrammarLanguage montiCoreGrammarLanguage = new MontiCoreGrammarLanguage();
        return montiCoreGrammarLanguage;
    }

    protected TaggingResolver createSymTabAndTaggingResolver(String modelPath) {
        ModelingLanguageFamily fam = new ModelingLanguageFamily();
        fam.addModelingLanguage(getMontiCoreGrammarLanguage());
        final ModelPath mp = new ModelPath(Paths.get(modelPath));
        GlobalScope scope = new GlobalScope(mp, fam);
        TaggingResolver tagging = new TaggingResolver(scope, Arrays.asList(modelPath));
        Features.registerTagTypes(tagging);

        return tagging;
    }

    @Before
    public void setup() {
        Log.getFindings().clear();
        Log.enableFailQuick(true);
    }

    @Test
    public void testArraySyntax(){
        String modelPath = "src/test/resources";
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver(modelPath);

        MCGrammarSymbol testGrammar = taggingResolver.<MCGrammarSymbol>resolve("grammar.TestGrammar", MCGrammarSymbol.KIND).orElse(null);
        assertNotNull(testGrammar);

        MCProdSymbol prod = taggingResolver.<MCProdSymbol>resolve("grammar.TestGrammar.Rule1", MCProdSymbol.KIND).orElse(null);
        assertNotNull(prod);
        List<TagSymbol> tags = (List<TagSymbol>) taggingResolver.getTags(prod, FeatureSymbol.KIND);
        assertEquals(1, tags.size());
    }

}
