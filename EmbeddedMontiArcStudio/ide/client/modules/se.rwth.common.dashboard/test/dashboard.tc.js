/* (c) https://github.com/MontiCore/monticore */
"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
const testcafe_1 = require("testcafe");
const constants_1 = require("../../se.rwth.common.test/constants");
const client_functions_1 = require("../../se.rwth.common.test/client.functions");
class DashboardTestController {
    constructor() {
        this.panel = testcafe_1.Selector("#dashboard-panel");
        this.list = testcafe_1.Selector("#dashboard-list");
        this.plus = testcafe_1.Selector("#dashboard-plus");
    }
    /*
     * Methods
     */
    async addProject(username, reponame, branchname) {
        return testcafe_1.ClientFunction((username, reponame, branchname) => Dashboard.addProject(username, reponame, branchname)).apply(null, arguments);
    }
    async removeProject(index) {
        return testcafe_1.ClientFunction((index) => Dashboard.removeProject(index, function () { })).apply(null, arguments);
    }
    async show() {
        return testcafe_1.ClientFunction(() => Dashboard.show()).apply(null, arguments);
    }
    async hide() {
        return testcafe_1.ClientFunction(() => Dashboard.hide()).apply(null, arguments);
    }
    /*
     * Checks
     */
    async checkVisibility(expectedVisibility, timeout = constants_1.Constants.DEFAULT_TIMEOUT) {
        const actualVisibility = await this.panel.getStyleProperty("display");
        return testcafe_1.t.expect(actualVisibility).eql(expectedVisibility, "[Dashboard] Visibility Violation", { timeout: timeout });
    }
    async checkShow(timeout) {
        return this.checkVisibility("block", timeout);
    }
    async checkHide(timeout) {
        return this.checkVisibility("none", timeout);
    }
    async checkProjects(expectedProjects, timeout) {
        const data = await client_functions_1.ClientFunctions.LocalStorage.getItem(DashboardTestController.KEY);
        const actualProjects = JSON.parse(data);
        return testcafe_1.t.expect(actualProjects).eql(expectedProjects, "[Dashboard] Projects Violation", { timeout: timeout });
    }
    /*
     * Actions
     */
    async clickPlus() {
        console.log("[Dashboard]: Plus has been clicked.");
        return testcafe_1.t.click(this.plus);
    }
}
DashboardTestController.KEY = "dashboard";
exports.DashboardTestController = DashboardTestController;
