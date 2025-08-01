/* (c) https://github.com/MontiCore/monticore */
"use strict";
var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : new P(function (resolve) { resolve(result.value); }).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
Object.defineProperty(exports, "__esModule", { value: true });
const testcafe_1 = require("testcafe");
class ClientFunctions {
}
ClientFunctions.LocalStorage = class {
    static getItem(key) {
        return __awaiter(this, arguments, void 0, function* () {
            return testcafe_1.ClientFunction((key) => window.localStorage.getItem(key)).apply(null, arguments);
        });
    }
    static setItem(key, value) {
        return __awaiter(this, arguments, void 0, function* () {
            return testcafe_1.ClientFunction((key, value) => window.localStorage.setItem(key, value)).apply(null, arguments);
        });
    }
};
exports.ClientFunctions = ClientFunctions;
