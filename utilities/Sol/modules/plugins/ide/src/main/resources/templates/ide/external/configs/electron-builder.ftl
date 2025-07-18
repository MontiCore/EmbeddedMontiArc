<#-- (c) https://github.com/MontiCore/monticore -->
${tc.signature("ide", "package")}
<#assign name = ide.getName()>
<#assign appId = package.getName().get()?replace("/", ".")?replace("@", "")>
productName: ${name}
appId: ${appId}

npmRebuild: false
electronVersion: 6.1.5

asar: false
directories:
  buildResources: build
  output: dist
files:
  - "!node_modules/**/*"
  - "lib/**/*"

win:
  icon: build/icon.ico
  target:
    - "nsis"

linux:
  icon: build/icons
  category: Development
  target:
    - "deb"

nsis:
  menuCategory: true
  oneClick: false
  perMachine: true
  installerHeaderIcon: build/icon.ico
  installerIcon: build/icon.ico
  uninstallerIcon: build/icon.ico
  installerSidebar: build/installerSidebar.bmp
  uninstallerSidebar: build/installerSidebar.bmp
  allowToChangeInstallationDirectory: true
  runAfterFinish: false
  artifactName: ${r"${productName}"}Installer.${r"${ext}"}

deb:
  artifactName: ${r"${productName}"}Installer.${r"${ext}"}
