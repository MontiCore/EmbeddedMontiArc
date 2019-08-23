/* (c) https://github.com/MontiCore/monticore */
import {Constants} from "../../se.rwth.common.test/constants";
import {DashboardTestController} from "./dashboard.tc";
import {LoaderTestController} from "../../se.rwth.common.loader/test/loader.tc";
import {BrowserDetectTestController} from "../../se.rwth.common.checker/test/browser-detect.tc";
import {DialogTestController} from "../../se.rwth.common.dialog/test/dialog.tc";

const url = Constants.HOST + "/modules/se.rwth.common.dashboard/test/dashboard.test.html";

fixture("Dashboard Test").page(url);

test("[1] Loader.hide", async () => {
    const loader = new LoaderTestController();
    const dialog = new DialogTestController();
    const browserDetect = new BrowserDetectTestController();

    const browser = await browserDetect.browser();

    if(browser.name === "chrome") await dialog.clickButton(0);

    await loader.checkHide();
});

test("[2] Dashboard.hide", async () => {
    const dashboard = new DashboardTestController();
    const dialog = new DialogTestController();
    const browserDetect = new BrowserDetectTestController();

    const browser = await browserDetect.browser();

    if(browser.name === "chrome") await dialog.clickButton(0);

    await dashboard.hide();
    await dashboard.checkHide();
});

test("[3] Dashboard.show", async () => {
    const dashboard = new DashboardTestController();
    const dialog = new DialogTestController();
    const browserDetect = new BrowserDetectTestController();

    const browser = await browserDetect.browser();

    if(browser.name === "chrome") await dialog.clickButton(0);

    await dashboard.hide();
    await dashboard.show();
    await dashboard.checkShow();
});

test("[4] Dashboard.addProject", async () => {
    const dashboard = new DashboardTestController();
    const dialog = new DialogTestController();
    const browserDetect = new BrowserDetectTestController();

    const browser = await browserDetect.browser();

    if(browser.name === "chrome") await dialog.clickButton(0);

    await dashboard.addProject("Peter", "APIs", "master");
    await dashboard.checkProjects([{ username: "Peter", reponame: "APIs", branchname: "master" }]);
});

test("[5] Dashboard.removeProject", async () => {
    const dashboard = new DashboardTestController();
    const dialog = new DialogTestController();
    const browserDetect = new BrowserDetectTestController();

    const browser = await browserDetect.browser();

    if(browser.name === "chrome") await dialog.clickButton(0);

    await dashboard.addProject("Peter", "APIs", "master");
    await dashboard.removeProject(0);
    await dashboard.checkProjects([]);
});
