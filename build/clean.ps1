# Edit config only
. ./config

function clean($item) {
  if(Test-Path -path $item) {
    Remove-Item -Recurse -Force $item
  }
}

clean $ext
clean $targetDir*
clean $targetName

if(!$cacheDownload) {
  clean $dl
  clean "curl"
  clean "7z"
}




