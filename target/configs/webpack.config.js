const path = require("path");
const fs = require("fs-extra");
const yargs = require('yargs');

const CopyWebpackPlugin = require("copy-webpack-plugin");

const rootPath = path.resolve(__dirname, "..", "..");
const modulePath = path.resolve(__dirname, "..");
const buildPath = path.join(modulePath, "build");

const baseConfig = require(path.join(modulePath, "webpack.config.js"));
const workerConfig = require(path.join(rootPath, "ide", "configs", "webpack.config.js"));

const indexFile = path.join(modulePath, "src-gen", "frontend", "index.js");

const { mode }  = yargs.option("mode", {
    description: "Mode to use",
    choices: ["development", "production"],
    default: "production"
}).argv;

const production = mode === "production";

if (production) {
    delete baseConfig.devtool;
    delete workerConfig.devtool;
}

let content = fs.readFileSync(indexFile).toString();

content = content.replace(
    "@elysium/core/lib/browser/messaging/messaging-frontend-module",
    "@elysium/core/lib/electron-browser/messaging/messaging-frontend-module"
);

content = content.replace(
    "@elysium/core/lib/browser/frontend-application-module",
    "@elysium/core/lib/electron-browser/frontend-application-module"
);

content = content.replace(
    "@elysium/core/lib/browser/logger-frontend-module",
    "@elysium/core/lib/electron-browser/logger-frontend-module"
);

content = content.replace(
    ".then(function () { return import('@elysium/dashboard/lib/browser/filesystem/filesystem-dashboard-frontend-module').then(load) })",
    ""
);

content = content.replace(
    ".then(function () { return Promise.resolve(require('@elysium/dashboard/lib/browser/filesystem/filesystem-dashboard-frontend-module')).then(load) })",
    ""
);

content = content.replace(
    "@theia/monaco/lib/browser/monaco-browser-module",
    "@emastudio/monaco/lib/electron-browser/monaco-electron-module"
);

fs.writeFileSync(indexFile, content);

workerConfig.output.path = baseConfig.output.path;

baseConfig.plugins.push(new CopyWebpackPlugin([
    {
        from: path.join(buildPath, "index.html")
    }
]));

module.exports = [baseConfig, workerConfig];