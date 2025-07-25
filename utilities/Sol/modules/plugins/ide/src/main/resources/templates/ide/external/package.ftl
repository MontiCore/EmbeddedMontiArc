<#-- (c) https://github.com/MontiCore/monticore -->
${tc.signature("package", "allPackages")}
<#assign name = package.getName().get()?replace("-common", "")>
<#assign version = package.getVersion().get()>
{
  "private": true,
  "name": "${name}-external",
  "version": "${version}",
  "files": ["src", "lib"],
  "main": "lib/main/index.js",
  "author": "SE-RWTH <kusmenko@se-rwth.de>", <#-- TODO: Make customizable. -->
  "homepage": "https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/utilities/Sol", <#-- TODO: Make customizable. -->
  "license": "SEE LICENSE IN LICENSE", <#-- TODO: Make customizable. -->
  "scripts": {
    "prepare": "yarn run clean && yarn run build && yarn run package",
    "clean": "rimraf lib && rimraf dist",
    "build": "webpack --config configs/webpack.config.js --mode development",
    "package": "electron-builder --config configs/electron-builder.yml"
  },
  "dependencies": {
    <#list allPackages as allPackage>
    <#assign dependencyName = allPackage.getName().get()>
    <#assign dependencyVersion = allPackage.getVersion().get()>
    "${dependencyName}": "${dependencyVersion}"<#if allPackage?has_next>,</#if>
    </#list>,
    "electron-debug": "^3.0.1",
    "express": "^4.17.1",
    "inversify": "^5.0.1",
    "reflect-metadata": "^0.1.13"
  },
  "devDependencies": {
    "css-loader": "0.28.11",
    "electron": "^6.0.7",
    "file-loader": "1.1.11",
    "html-webpack-plugin": "^3.2.0",
    "svg-inline-loader": "^0.8.0",
    "ts-loader": "^6.0.4",
    "webpack": "^4.41.2",
    "webpack-cli": "^3.1.1",
    "electron-builder": "^21.2.0"
  }
}
