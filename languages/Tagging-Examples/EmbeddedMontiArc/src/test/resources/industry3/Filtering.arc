/* (c) https://github.com/MontiCore/monticore */
package industry3;

component Filtering {
  ports in Integer omega,
        in Integer windSpeed,
        out Integer filteredOmega,
        out Integer filteredWindSpeed;
}
