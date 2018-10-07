package taggingTest; /**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */

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
 * @author Michael von Wenckstern
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
        //assertNotNull(testGrammar);

        MCProdSymbol prod = taggingResolver.<MCProdSymbol>resolve("grammar.TestGrammar.Rule1", MCProdSymbol.KIND).orElse(null);
        // assertNotNull(prod);
        List<TagSymbol> tags = (List<TagSymbol>) taggingResolver.getTags(prod, FeatureSymbol.KIND);
        assertEquals(1, tags.size());
    }

}
