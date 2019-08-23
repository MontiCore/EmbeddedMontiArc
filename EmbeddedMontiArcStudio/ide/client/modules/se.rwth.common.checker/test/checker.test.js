/* (c) https://github.com/MontiCore/monticore */
"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
const constants_1 = require("../../se.rwth.common.test/constants");
const settings_tc_1 = require("../../se.rwth.common.settings/test/settings.tc");
const browser_detect_tc_1 = require("./browser-detect.tc");
const dialog_tc_1 = require("../../se.rwth.common.dialog/test/dialog.tc");
const url = constants_1.Constants.HOST + "/modules/se.rwth.common.checker/test/checker.test.html";
fixture("Checker Test").page(url);
test("[1] Checker.check", async () => {
    const browserDetect = new browser_detect_tc_1.BrowserDetectTestController();
    const settings = new settings_tc_1.SettingsTestController();
    const dialog = new dialog_tc_1.DialogTestController();
    const browser = await browserDetect.browser();
    const chromeMessage = 'The system has detected that you are using Google Chrome as web browser. ' +
        'It is highly recommended that you activate <span class="highlighted">WebAssembly ' +
        'Structured Cloning Support</span> in order to speed up the preparation process. You can ' +
        'activate it by entering <span class="highlighted">chrome://flags#enable-webassembly</span> ' +
        'as URL and switching the option to "Enabled".';
    const edgeMessage = 'The system has detected that you are using Microsoft Edge as web browser. ' +
        'It is absolutely necessary that you activate WebAssembly in your web browser in order to ' +
        'make use of the services provided by this website. You can do this by entering' +
        '<span class="highlighted">about:flags</span> as URL an enabling the corresponding option.';
    if (browser.name === "chrome") {
        await dialog.checkShow(chromeMessage, "warning", ["OK, never show again", "OK"]);
        await dialog.clickButton(0);
    }
    if (browser.name === "edge" && browser.version < "16") {
        await dialog.checkShow(edgeMessage, "error", []);
    }
    else {
        await settings.checkValue("flags.prerequisites", true);
    }
});
