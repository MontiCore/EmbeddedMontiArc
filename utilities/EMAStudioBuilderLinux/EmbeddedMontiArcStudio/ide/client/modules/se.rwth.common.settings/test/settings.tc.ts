/* (c) https://github.com/MontiCore/monticore */
import {ClientFunction, t} from "testcafe";
import {Constants} from "../../se.rwth.common.test/constants";
import {ClientFunctions} from "../../se.rwth.common.test/client.functions";

declare var Settings;

export class SettingsTestController {
    protected static readonly KEY: string = "settings";

    /*
     * Methods
     */
    public async set(key: string, value: string | any): Promise<void> {
        return ClientFunction((key, value) => Settings.set(key, value)).apply(null, arguments);
    }

    public async get(key: string): Promise<any> {
        return ClientFunction((key) => Settings.get(key)).apply(null, arguments);
    }

    /*
     * Checks
     */
    public async checkValue(key: string, expectedValue: any, timeout: number = Constants.DEFAULT_TIMEOUT): Promise<void> {
        const data = await ClientFunctions.LocalStorage.getItem(SettingsTestController.KEY) || "{}";
        const settings = JSON.parse(data);
        const actualValue = settings[key];

        return t.expect(actualValue).eql(expectedValue, "[Settings]: Value Violation", { timeout: timeout});
    }
}
