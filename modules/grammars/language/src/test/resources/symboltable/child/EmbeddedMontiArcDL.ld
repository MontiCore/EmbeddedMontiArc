/* (c) https://github.com/MontiCore/monticore */
package child;

import child.parents.*;

language EmbeddedMontiArcDL extends EmbeddedMontiArc, EmbeddedMontiArcMath {
    declare template emadlComponent for "EmbeddedMontiArcDL.njk" {
        attribute label = "EmbeddedMontiArcDL Component";

        /*
         * Package
         */
        option packageName:string {
            prop label = "Package";
            prop required = true;
        }

        /*
         * Component
         */
        option componentName:string {
            prop label = "Component";
            prop required = true;
        }

        /*
         * Ports
         */
        option ports:list {
            prop label = "Add Port";

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

    undeclare template emamComponent;
}
