/* (c) https://github.com/MontiCore/monticore */
package industry2;

component PitchEstimator {
  ports in Integer windSpeed,
        in Integer rotorSpeed,
        out Integer estimatedAngle;
}
