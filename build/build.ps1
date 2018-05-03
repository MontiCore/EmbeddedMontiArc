$dl = ".\download\"       
$ext = ".\extracted\"  

# The Folder which will be present after self-extracting
$targetDir = "EMAStudio"
# The name of the self-extracting archive
$targetName = "EmbeddedMontiArcStudio.exe"
# Project items or dirs that need to be included
$packingItems = @("ide.bat", "EmbeddedMontiArcStudio")

. ./download.ps1 $dl

. ./extract.ps1 $dl $ext

. ./pack.ps1 $targetDir $targetName $packingItems
