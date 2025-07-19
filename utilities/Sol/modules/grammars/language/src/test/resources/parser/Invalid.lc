/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.embeddedmontiarc

import some.qualified.Name

language EmbeddedMontiArc {
    declare template emaComponent for "EmbeddedMontiArcComponent.njk" {
        attribute label  "EmbeddedMontiArc Component";

        /*
         * Package
         */
        option packageName:string {
            prop label = "Package"
            prop required = true;
        }

        /*
         * Component
         */
        option component:string {
            prop label  "Component";
            prop required = true;
        }

        /*
         * Ports
         */
        option ports:list {
            prop label = "Add Port"

            option direction:string {
                prop label = "Direction";
                prop required = true;
            }

            option type:string {
                prop label = "Type";
                prop required = true;
            }

            option name:string {
                prop label = "Name";
                prop required = true;
            }
        }
    }

    undeclare template someTemplate
}
