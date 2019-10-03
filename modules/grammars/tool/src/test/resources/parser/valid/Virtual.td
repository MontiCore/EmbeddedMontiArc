/*
 * (c) https://github.com/MontiCore/monticore
 */
package parser;

import parser.Invalid;
import parser.Imaginary;

virtual tool Virtual {
     attribute command = "java -jar env{VIRTUAL}";
     attribute alias = VIRTUAL in ENV;
     attribute path = "some/imaginary/path";
     attribute environment = Imaginary;

     resource html {
         attribute alias = HTMLFile in ENV;
         attribute path = "some/imaginary/path";
         attribute platform = "win32";
     }

     resource css {
         attribute alias = CSSFile in ENV;
         attribute path = "some/imaginary/path";
     }
}