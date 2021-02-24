/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.schemalang;

schema EMATypes {
    natural_with_zero = 0: N
    natural_without_zero = 1: N1
    boolean_type = true: B
    integer = -1: Z
    rational = 0.25: Q
    one_dimensional_matrix: Q(0:1)^{10}
    three_dimensional_matrix: Z(0:255)^{1,28,28}
}