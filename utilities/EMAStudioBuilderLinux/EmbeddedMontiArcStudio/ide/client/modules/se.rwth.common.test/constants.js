/* (c) https://github.com/MontiCore/monticore */
"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
const Util = require("util");
class Constants {
    static getTravisHost() {
        const slug = process.env.TRAVIS_REPO_SLUG;
        const parts = slug.split('/');
        return Util.format("https://%s.github.io/%s", parts[0], parts[1]);
    }
}
Constants.HOST = process.env.HOST ? process.env.HOST : Constants.getTravisHost();
Constants.DEFAULT_TIMEOUT = 3000;
exports.Constants = Constants;
