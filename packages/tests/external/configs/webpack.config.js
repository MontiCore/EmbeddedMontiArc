/*
 * (c) https://github.com/MontiCore/monticore
 */
const path = require("path");
const HTMLWebpackPlugin = require("html-webpack-plugin");

const rendererModule = {
    entry: "./src/renderer/index.ts",
    target: "electron-renderer",
    context: path.resolve(__dirname, ".."),
    module: {
        rules: [
            {
                test: /\.tsx?$/,
                use: [
                    {
                        loader: "ts-loader",
                        options: {
                            onlyCompileBundledFiles: true,
                            configFile: "compile.tsconfig.json"
                        }
                    }
                ],
                exclude: /node_modules/,
            },
            {
                test: /\.css$/i,
                use: ["style-loader", "css-loader"]
            },
            {
                test: /\.svg$/,
                loader: "svg-inline-loader"
            },
            {
                test: /\.(png|svg|jpg|gif|woff|woff2)$/,
                use: ["file-loader"]
            }
        ]
    },
    resolve: {
        extensions: [".tsx", ".ts", ".js"]
    },
    output: {
        filename: "index.js",
        path: path.resolve(__dirname, "..", "lib", "renderer")
    },
    plugins: [
        new HTMLWebpackPlugin({ title: "External Test" })
    ]
};

const mainModule = {
    entry: "./src/main/index.ts",
    target: "electron-main",
    context: path.resolve(__dirname, ".."),
    module: {
        rules: [
            {
                test: /\.tsx?$/,
                use: [
                    {
                        loader: "ts-loader",
                        options: {
                            onlyCompileBundledFiles: true,
                            configFile: "compile.tsconfig.json"
                        }
                    }
                ],
                exclude: /node_modules/,
            }
        ]
    },
    resolve: {
        extensions: [".tsx", ".ts", ".js"]
    },
    output: {
        filename: "index.js",
        path: path.resolve(__dirname, "..", "lib", "main")
    }
};

module.exports = [rendererModule, mainModule];