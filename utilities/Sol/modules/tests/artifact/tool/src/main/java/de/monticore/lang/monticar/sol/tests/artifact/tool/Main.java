/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.tests.artifact.tool;

public class Main {
    public static void main(String[] args) throws InterruptedException {
        long iteration = 0;
        String text = args[0];
        int delay = 1000 * Integer.parseInt(args[1]);

        while (true) {
            System.out.println(String.format("%s %d", text, ++iteration));
            Thread.sleep(delay);
        }
    }
}
