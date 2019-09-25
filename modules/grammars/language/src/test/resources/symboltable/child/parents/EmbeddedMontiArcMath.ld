/*
 * (c) https://github.com/MontiCore/monticore
 */
package child.parents;

language EmbeddedMontiArcMath {
    declare template emamComponent for "EmbeddedMontiArcMath.njk" {
        attribute label = "EmbeddedMontiArcMath Component";

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
}
