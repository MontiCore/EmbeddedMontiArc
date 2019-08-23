/* (c) https://github.com/MontiCore/monticore */
import {Constants} from "../../se.rwth.common.test/constants";
import {SettingsTestController} from "./settings.tc";

const url = Constants.HOST + "/modules/se.rwth.common.settings/test/settings.test.html";

fixture("Settings Test").page(url);

test("[1] Settings.set", async () => {
    const settings = new SettingsTestController();
    const value = { parent: { child: "test" } };

    await settings.set("test", value);
    await settings.checkValue("test", value);
});

test("[2] Settings.get", async(testController: TestController) => {
    const settings = new SettingsTestController();
    const value = { parent: { child: "test" } };

    await settings.set("test", value);
    await testController.navigateTo(url);

    const expectedValue = await settings.get("test");

    await settings.checkValue("test", expectedValue);
});
