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
 * @author Michael von Wenckstern
 */
public class TagTest {

    protected static CD4AnalysisLanguage getMontiArcLanguage() {
        CD4AnalysisLanguage montiArcLanguage = new CD4AnalysisLanguage();
        return montiArcLanguage;
    }

    protected TaggingResolver createSymTabAndTaggingResolver(String modelPath) {
        ModelingLanguageFamily fam = new ModelingLanguageFamily();
        fam.addModelingLanguage(getMontiArcLanguage());
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
