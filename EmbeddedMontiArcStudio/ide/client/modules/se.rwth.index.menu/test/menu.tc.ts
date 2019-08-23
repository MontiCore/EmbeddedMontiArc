/* (c) https://github.com/MontiCore/monticore */
import {AbstractModuleTestController} from "../framework/ModuleTestController";
import {Selector, t} from "testcafe";

export class MenuTestController extends AbstractModuleTestController {
    protected menu: Selector;
    protected demo: Selector;

    public constructor() {
        super();
        this.menu = Selector("#menu-panel");
        this.demo = Selector("#menu-item-demo");
    }

    /*
     * Actions
     */
    public async clickFromDemo(): Promise<void> {
        return t.click(this.demo);
    }
}
