# (c) https://github.com/MontiCore/monticore  
$date = Get-Date -format "ddMMM-HHmm"
# The Folder which will be present after self-extracting
$targetDir = "EMAStudio"
# The name of the self-extracting archive
$targetName = "EmbeddedMontiArcStudio.exe"
# Project items or dirs that need to be included
$packingItems = @("ide.bat", "EmbeddedMontiArcStudio")
# Set to 0 to remove downloads in clean phase
$cacheDownload = 1



$dl = ".\download\"       
$ext = ".\extracted\"  
$url7z = "https://github.com/EmbeddedMontiArc/EMAStudioBuilder/releases/download/tools/7ZipPortable.zip"
$urlcurl =  "https://bintray.com/artifact/download/vszakats/generic/curl-7.59.0-win32-mingw.zip"
$build = [System.IO.Path]::GetFullPath('.\')
$curl = (Join-Path $build 'curl\curl.exe')
$7z = (Join-Path $build "7z\7z.exe")



