/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.mathopt._symboltable;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.math.MathSymbolTableCreatorTest;
import de.monticore.lang.math._symboltable.MathLanguage;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import org.junit.BeforeClass;

import java.nio.file.Paths;

/**
 * Tests all Math Symbols with the new SymbolTableCreator.
 */
public class MathOptSymbolTableCreatorTest extends MathSymbolTableCreatorTest {

    @BeforeClass
    public static void setUpClass() {
        // only create symTab once
        symTab = createSymTab("src/test/resources");
    }

    protected static Scope createSymTab(String... modelPath) {
        ModelingLanguageFamily fam = new ModelingLanguageFamily();
        fam.addModelingLanguage(new MathLanguage());
        fam.addModelingLanguage(new MathOptLanguage());
        final ModelPath mp = new ModelPath();

        for (String m : modelPath) {
            mp.addEntry(Paths.get(m));
        }

        GlobalScope scope = new GlobalScope(mp, fam);
        return scope;
    }

}
