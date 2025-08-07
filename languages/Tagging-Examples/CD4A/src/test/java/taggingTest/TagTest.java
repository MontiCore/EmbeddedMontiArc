/* (c) https://github.com/MontiCore/monticore */
package taggingTest;

import java.nio.file.Paths;
import java.util.*;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.tagging._symboltable.TagSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.GlobalScope;
import de.monticore.umlcd4a.CD4AnalysisLanguage;
import de.monticore.umlcd4a.symboltable.CDFieldSymbol;
import de.monticore.umlcd4a.symboltable.CDSymbol;
import de.monticore.umlcd4a.symboltable.CDTypeSymbol;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;

/**
 * Created by Michael von Wenckstern on 30.05.2016.
 *
 */
public class TagTest {

    protected static CD4AnalysisLanguage getCD4ALanguage() {
        CD4AnalysisLanguage cd4AnalysisLanguage = new CD4AnalysisLanguage();
        return cd4AnalysisLanguage;
    }

    protected TaggingResolver createSymTabAndTaggingResolver(String modelPath) {
        ModelingLanguageFamily fam = new ModelingLanguageFamily();
        fam.addModelingLanguage(getCD4ALanguage());
        final ModelPath mp = new ModelPath(Paths.get(modelPath));
        GlobalScope scope = new GlobalScope(mp, fam);
        TaggingResolver tagging = new TaggingResolver(scope, Arrays.asList(modelPath));
        CD4ATags.registerTagTypes(tagging);

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

        CDSymbol myClassdiagramm = taggingResolver.<CDSymbol>resolve("cd4a.MyClassDiagramm", CDSymbol.KIND).orElse(null);
        assertNotNull(myClassdiagramm);

        CDTypeSymbol myClass = taggingResolver.<CDTypeSymbol>resolve("cd4a.MyClassDiagramm.MyClass", CDTypeSymbol.KIND).orElse(null);
        assertNotNull(myClass);
        List<TagSymbol> tags = (List<TagSymbol>) taggingResolver.getTags(myClass, ClassTagSymbol.KIND);
        assertEquals(1, tags.size());
        String tagValue = ((ClassTagSymbol) tags.get(0)).getValue();
        assertEquals("A String tagged to the class MyClass", tagValue);

        CDFieldSymbol myVar = taggingResolver.<CDFieldSymbol>resolve("cd4a.MyClassDiagramm.MyClass.myVar", CDFieldSymbol.KIND).orElse(null);
        assertNotNull(myVar);
        tags = (List<TagSymbol>) taggingResolver.getTags(myVar, VariableTagSymbol.KIND);
        assertEquals(1, tags.size());
        tagValue = ((VariableTagSymbol) tags.get(0)).getValue();
        assertEquals("A String tagged to the variable myVar", tagValue);

    }

}
