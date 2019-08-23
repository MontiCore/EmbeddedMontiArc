/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emscripten;

/**
 * Optimization levels for emscripten. Higher optimization levels introduce progressively more
 * optimization, resulting in improved performance and code size at the cost of increased
 * compilation time.
 *
 * @see <a href="https://kripken.github.io/emscripten-site/docs/optimizing/Optimizing-Code.html">https://kripken.github.io/emscripten-site/docs/optimizing/Optimizing-Code.html</a>
 */
public enum Optimization {
  /**
   * No optimization
   */
  O0("0"),

  /**
   * Small optimizations that take little time
   */
  O1("1"),

  /**
   * More optimizations.
   */
  O2("2"),

  /**
   * Optimize for size. Enables {@link #O2} optimizations that typically do not increase code size.
   * It also tries to apply measures to further reduce code size.
   */
  Os("s"),

  /**
   * Similar to {@link #O2} and reduces code size further than {@link #Os}, which comes with a
   * trade-off in performance.
   */
  Oz("z"),

  /**
   * Most aggressive optimization, that typically take significantly longer than {@link #O2}.
   */
  O3("3");

  private final String level;

  Optimization(String level) {

    this.level = level;
  }

  @Override
  public String toString() {
    return "-O" + level;
  }
}
