package de.monticore.lang.montisim.simlang;

import de.monticore.lang.montisim.simlang.adapter.SimLangContainer;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

public class SimLangContainerTest {
  @Before
  public void setUp() {
    // ensure an empty log
    Log.getFindings().clear();
    Log.enableFailQuick(false);
  }

  @Test
  public void testSimLangContainer() {
    final SimLangContainer adapter = SimLangTool.parseIntoContainer("src/test/resources/test/ast/ASTTest.sim");

    assert adapter.getSimulationRenderFrequency().isPresent();
    assert adapter.getSimulationRenderFrequency().get().getNUnit().get().getNumberUnit().equals("60.0ms");
    //assert adapter.getSim_render_frequency().get().getNUnit().get().getNumberUnit().equals("60ms");
  }
}
