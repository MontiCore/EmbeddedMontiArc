# (c) https://github.com/MontiCore/monticore  
# Edit config only
. ./config

function clean($item) {
  if(Test-Path -path $item) {
    Remove-Item -Recurse -Force $item
  }
}

clean $targetDir*
clean $targetName

if(!$cacheDownload) {
  clean $ext
  clean $dl
  clean "curl"
  clean "7z"
}
