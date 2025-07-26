/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc;

import de.monticore.lang.embeddedmontiarc.helper.AutoconnectMode;

import java.io.File;

public final class EmbeddedMontiArcConstants {

  /**
   * Default auto connect mode = OFF.
   */
  public static final AutoconnectMode DEFAULT_AUTO_CONNECT = AutoconnectMode.OFF;

  /**
   * Default documentation directory.
   */
  public static final String DEFAULT_DOC_DIR = "target" + File.separator + "madoc";

  /**
   * Default output directory.
   */
  public static final String DEFAULT_GEN_DIR = "target" + File.separator +
      "generated-sources" + File.separator +
      "montiarc" + File.separator +
      "sourcecode";

  /**
   * Default model directory.
   */
  public static final String DEFAULT_MODEL_DIR = "src" + File.separator + "main" + File.separator
      + "models";

}
