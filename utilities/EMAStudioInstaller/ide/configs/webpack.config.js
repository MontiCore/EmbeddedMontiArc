// @ts-check
const path = require("path");
const yargs = require('yargs');

const CopyWebpackPlugin = require("copy-webpack-plugin");

const rootPath = path.resolve(__dirname, "..");
const genPath = path.join(rootPath, "extensions", "**", "gen", "*.*");

const { mode }  = yargs.option("mode", {
    description: "Mode to use",
    choices: ["development", "production"],
    default: "production"
}).argv;

const workerConfig = {
    entry: path.resolve(rootPath, "build", "worker.js"),
    context: rootPath,
    output: {
        filename: "bundle.worker.js"
    },
    target: "webworker",
    mode,
    node: {
        fs: "empty",
        child_process: "empty",
        net: "empty",
        crypto: "empty"
    },
    module: {
        rules: []
    },
    resolve: {
        extensions: [".js"]
    },
    devtool: "source-map",
    plugins: [
        new CopyWebpackPlugin([
            {
                from: genPath,
                to: '.'
            }
        ])
    ],
    stats: {
        warnings: true
    }
};

module.exports = workerConfig;