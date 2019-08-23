/* (c) https://github.com/MontiCore/monticore */
"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
const constants_1 = require("../../se.rwth.common.test/constants");
const settings_tc_1 = require("./settings.tc");
const url = constants_1.Constants.HOST + "/modules/se.rwth.common.settings/test/settings.test.html";
fixture("Settings Test").page(url);
test("[1] Settings.set", async () => {
    const settings = new settings_tc_1.SettingsTestController();
    const value = { parent: { child: "test" } };
    await settings.set("test", value);
    await settings.checkValue("test", value);
});
test("[2] Settings.get", async (testController) => {
    const settings = new settings_tc_1.SettingsTestController();
    const value = { parent: { child: "test" } };
    await settings.set("test", value);
    await testController.navigateTo(url);
    const expectedValue = await settings.get("test");
    await settings.checkValue("test", expectedValue);
});
