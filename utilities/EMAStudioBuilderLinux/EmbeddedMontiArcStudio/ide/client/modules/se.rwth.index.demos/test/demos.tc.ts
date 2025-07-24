/* (c) https://github.com/MontiCore/monticore */
import {AbstractModuleTestController} from "../framework/ModuleTestController";
import {Selector, t} from "testcafe";
import * as Util from "util";

export class DemosTestController extends AbstractModuleTestController {
    protected panel: Selector;
    protected list: Selector;
    protected back: Selector;

    public constructor() {
        super();
        this.panel = Selector("#demos-panel");
        this.list = Selector("#demos-list");
        this.back = Selector("#demos-back");
    }

    /*
     * Actions
     */
    public async clickBack(): Promise<void> {
        return t.click(this.back);
    }

    public async clickDemo(index: number): Promise<void> {
        const message = Util.format("[Demos]: Demo at Position %d has been clicked.", index);
        const demos = this.list.child("li");
        const demo = demos.nth(index + 1);

        console.log(message);
        return t.click(demo);
    }
}
