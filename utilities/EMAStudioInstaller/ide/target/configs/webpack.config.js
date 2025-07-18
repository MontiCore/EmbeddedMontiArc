const path = require("path");
const yargs = require('yargs');

const CopyWebpackPlugin = require("copy-webpack-plugin");

const rootPath = path.resolve(__dirname, "..", "..");
const modulePath = path.resolve(__dirname, "..");
const buildPath = path.join(modulePath, "build");

const baseConfig = require(path.join(modulePath, "webpack.config.js"));
const workerConfig = require(path.join(rootPath, "configs", "webpack.config.js"));

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

workerConfig.output.path = baseConfig.output.path;

baseConfig.plugins.push(new CopyWebpackPlugin([
    {
        from: path.join(buildPath, "index.html")
    },
    {
        from: path.join(buildPath, "favicon.ico")
    }
]));

module.exports = [baseConfig, workerConfig];