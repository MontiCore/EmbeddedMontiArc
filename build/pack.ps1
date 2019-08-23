# (c) https://github.com/MontiCore/monticore  
$targetDir = "$targetDir.$date"

if(!(Test-Path -path $targetDir)) {
  New-Item -ItemType directory -Path $targetDir
}  

"Copy extracted dependencies to build"
Get-ChildItem -Path $ext | Copy-Item -Destination $targetDir -Recurse -Force


"Copy project items to build"
foreach ($item in $packingItems) {
    $item = "..\" + $item
    Copy-Item -Recurse -Force $item -Destination $targetDir	
}


"Packing to self-extracting exe"
&$7z a $targetName -sfx -t7z -m0=lzma2 -mx5 $targetDir | Out-Null
