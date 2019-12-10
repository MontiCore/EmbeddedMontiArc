${tc.signature("ide", "package", "allPackages")}
<#assign packageName = package.getName().get()?replace("-common", "")>
<#assign packageVersion = package.getVersion().get()>
<#assign ideName = ide.getName()>
{
  "private": true,
  "name": "${packageName}-internal",
  "version": "${packageVersion}",
  "scripts": {
    "prepare": "sol theia build",
    "clean": "theia clean && rimraf webpack.config++.js",
    "start": "sol theia start"
  },
  "dependencies": {
    <#list allPackages as allPackage>
    <#assign dependencyName = allPackage.getName().get()>
    <#assign dependencyVersion = allPackage.getVersion().orElse("latest")>
    "${dependencyName}": "^${dependencyVersion}"<#if allPackage?has_next>,</#if>
    </#list>
  },
  "devDependencies": {
    "@embeddedmontiarc/sol-development-cli": "^2019.12.10-SNAPSHOT"
  },
  "theia": {
    "frontend": {
      "config": {
        "applicationName": "${ideName}",
        "preferences": {
          "files.enableTrash": false
        }
      }
    }
  },
  "sol": {
    "directories": {
      "models": "sol/models"
    }
  }
}
