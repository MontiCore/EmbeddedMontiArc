${tc.signature("ide", "package", "allPackages", "resolutions")}
<#assign packageName = package.getName().get()?replace("-common", "")>
<#assign packageVersion = package.getVersion().get()>
<#assign ideName = ide.getName()>
{
  "private": true,
  "name": "${packageName}-internal",
  "version": "${packageVersion}",
  "files": ["src", "src-gen", "lib", "sol"],
  "scripts": {
    "prepare": "sol theia build",
    "clean": "theia clean && rimraf webpack.config++.js",
    "start": "sol theia start"
  },
  "dependencies": {
    "@embeddedmontiarc/sol-external-monaco": ">=2020.4.29-SNAPSHOT",
    <#list allPackages as allPackage>
    <#assign dependencyName = allPackage.getName().get()>
    <#assign dependencyVersion = allPackage.getVersion().orElse("latest")>
    "${dependencyName}": "${dependencyVersion}"<#if allPackage?has_next>,</#if>
    </#list>
  },
  "resolutions": {
    <#list resolutions.keySet() as resolutionPackage>
    <#assign resolutionVersion = resolutions.getString(resolutionPackage)>
    "${resolutionPackage}": "${resolutionVersion}"<#if resolutionPackage?has_next>,</#if>
    </#list>
  },
  "devDependencies": {
    "@theia/cli": "latest",
    "@embeddedmontiarc/sol-development-cli": ">=2020.4.29-SNAPSHOT"
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
