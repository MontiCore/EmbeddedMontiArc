/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang;

import de.monticore.lang.montisim.simlang.adapter.SLMontiSimAdapter;
import de.monticore.lang.montisim.simlang.adapter.SimLangContainer;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

public class MontiSimAdapterTest {
  @Before
  public void setUp() {
    // ensure an empty log
    Log.getFindings().clear();
    Log.enableFailQuick(false);
  }

  @Test
  public void testSimLangContainer() {
    final SimLangContainer container = SimLangTool.parseIntoContainer("src/test/resources/test/adapter/Advanced.sim");
    final SLMontiSimAdapter adapter = new SLMontiSimAdapter(container);

    assert adapter.getMapName().equals("Advanced");
  }
}
