New-Item -ItemType directory -Path $targetDir


"Copy extracted dependencies to build"
Copy-Item -Recurse -Force -Path ".\extracted\" -Filter * -Destination $targetDir 


"Copy project items to build"
foreach ($item in $packingItems) {
    $item = "..\" + $item
    Copy-Item -Recurse -Force "$item -Destination $targetDir 	
}


"Packing to self-extracting exe"
.\7za.exe a $targetName -sfx -t7z -m0=lzma2 -mx5 $targetDir
