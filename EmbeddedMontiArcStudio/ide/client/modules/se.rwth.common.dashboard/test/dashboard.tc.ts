/* (c) https://github.com/MontiCore/monticore */
import {ClientFunction, Selector, t} from "testcafe";
import {Constants} from "../../se.rwth.common.test/constants";
import {ClientFunctions} from "../../se.rwth.common.test/client.functions";

declare var Dashboard;

export class DashboardTestController {
    protected static readonly KEY: string = "dashboard";

    protected panel: Selector;
    protected list: Selector;
    protected plus: Selector;

    public constructor() {
        this.panel = Selector("#dashboard-panel");
        this.list = Selector("#dashboard-list");
        this.plus = Selector("#dashboard-plus");
    }

    /*
     * Methods
     */
    public async addProject(username: string, reponame: string, branchname: string): Promise<void> {
        return ClientFunction((username, reponame, branchname) => Dashboard.addProject(username, reponame, branchname)).apply(null, arguments);
    }

    public async removeProject(index: number): Promise<void> {
        return ClientFunction((index) => Dashboard.removeProject(index, function() {})).apply(null, arguments);
    }

    public async show(): Promise<void> {
        return ClientFunction(() => Dashboard.show()).apply(null, arguments);
    }

    public async hide(): Promise<void> {
        return ClientFunction(() => Dashboard.hide()).apply(null, arguments);
    }

    /*
     * Checks
     */
    protected async checkVisibility(expectedVisibility: string, timeout: number = Constants.DEFAULT_TIMEOUT): Promise<void> {
        const actualVisibility = await this.panel.getStyleProperty("display");

        return t.expect(actualVisibility).eql(expectedVisibility, "[Dashboard] Visibility Violation", { timeout: timeout });
    }

    public async checkShow(timeout?: number): Promise<void> {
        return this.checkVisibility("block", timeout);
    }

    public async checkHide(timeout?: number): Promise<void> {
        return this.checkVisibility("none", timeout);
    }

    public async checkProjects(expectedProjects: any, timeout?: number): Promise<void> {
        const data = await ClientFunctions.LocalStorage.getItem(DashboardTestController.KEY);
        const actualProjects = JSON.parse(data);

        return t.expect(actualProjects).eql(expectedProjects, "[Dashboard] Projects Violation", { timeout: timeout });
    }

    /*
     * Actions
     */
    public async clickPlus(): Promise<void> {
        console.log("[Dashboard]: Plus has been clicked.");
        return t.click(this.plus);
    }
}
