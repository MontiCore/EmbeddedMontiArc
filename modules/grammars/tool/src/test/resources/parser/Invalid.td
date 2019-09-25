/*
 * (c) https://github.com/MontiCore/monticore
 */
package parser;

import parser.Invalid;
import parser.Imaginary;

external tool Valid as VALID in ENV extends Invalid, Imaginary {
    command = "java -jar env{VALID}";

    external resource html as HTMLFile for "some/imaginary/path" {
        attribute platform = "win32";
    }

    resource css as CSSFile in ENV;
}