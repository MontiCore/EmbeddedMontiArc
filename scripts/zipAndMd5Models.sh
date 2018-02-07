#!/bin/bash
createZip() {
  cd $1
  zip -R ../models.zip '*.m' '*.ema' '*.emam'
  ema="../models.zip"
  md5=`md5sum ${ema} | awk '{ print $1 }'`
  cd ..
  mv "models.zip" "models$md5.zip"
  return $md5
}