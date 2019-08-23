/* (c) https://github.com/MontiCore/monticore */
"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
const constants_1 = require("../../se.rwth.common.test/constants");
const dashboard_tc_1 = require("./dashboard.tc");
const loader_tc_1 = require("../../se.rwth.common.loader/test/loader.tc");
const browser_detect_tc_1 = require("../../se.rwth.common.checker/test/browser-detect.tc");
const dialog_tc_1 = require("../../se.rwth.common.dialog/test/dialog.tc");
const url = constants_1.Constants.HOST + "/modules/se.rwth.common.dashboard/test/dashboard.test.html";
fixture("Dashboard Test").page(url);
test("[1] Loader.hide", async () => {
    const loader = new loader_tc_1.LoaderTestController();
    const dialog = new dialog_tc_1.DialogTestController();
    const browserDetect = new browser_detect_tc_1.BrowserDetectTestController();
    const browser = await browserDetect.browser();
    if (browser.name === "chrome")
        await dialog.clickButton(0);
    await loader.checkHide();
});
test("[2] Dashboard.hide", async () => {
    const dashboard = new dashboard_tc_1.DashboardTestController();
    const dialog = new dialog_tc_1.DialogTestController();
    const browserDetect = new browser_detect_tc_1.BrowserDetectTestController();
    const browser = await browserDetect.browser();
    if (browser.name === "chrome")
        await dialog.clickButton(0);
    await dashboard.hide();
    await dashboard.checkHide();
});
test("[3] Dashboard.show", async () => {
    const dashboard = new dashboard_tc_1.DashboardTestController();
    const dialog = new dialog_tc_1.DialogTestController();
    const browserDetect = new browser_detect_tc_1.BrowserDetectTestController();
    const browser = await browserDetect.browser();
    if (browser.name === "chrome")
        await dialog.clickButton(0);
    await dashboard.hide();
    await dashboard.show();
    await dashboard.checkShow();
});
test("[4] Dashboard.addProject", async () => {
    const dashboard = new dashboard_tc_1.DashboardTestController();
    const dialog = new dialog_tc_1.DialogTestController();
    const browserDetect = new browser_detect_tc_1.BrowserDetectTestController();
    const browser = await browserDetect.browser();
    if (browser.name === "chrome")
        await dialog.clickButton(0);
    await dashboard.addProject("Peter", "APIs", "master");
    await dashboard.checkProjects([{ username: "Peter", reponame: "APIs", branchname: "master" }]);
});
test("[5] Dashboard.removeProject", async () => {
    const dashboard = new dashboard_tc_1.DashboardTestController();
    const dialog = new dialog_tc_1.DialogTestController();
    const browserDetect = new browser_detect_tc_1.BrowserDetectTestController();
    const browser = await browserDetect.browser();
    if (browser.name === "chrome")
        await dialog.clickButton(0);
    await dashboard.addProject("Peter", "APIs", "master");
    await dashboard.removeProject(0);
    await dashboard.checkProjects([]);
});
