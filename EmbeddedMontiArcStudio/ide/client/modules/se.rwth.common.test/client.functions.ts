/* (c) https://github.com/MontiCore/monticore */
import {ClientFunction} from "testcafe";

export class ClientFunctions {
    public static readonly LocalStorage = class {
        public static async getItem(key: string): Promise<string> {
            return ClientFunction((key) => window.localStorage.getItem(key)).apply(null, arguments);
        }

        public static async setItem(key: string, value: string): Promise<void> {
            return ClientFunction((key, value) => window.localStorage.setItem(key, value)).apply(null, arguments);
        }
    };
}
