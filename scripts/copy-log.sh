# script to copy the training log
DATE=$(date +%d-%b-%Y_%T)
cp train.log logs/history/$1_[$DATE].log
cp train.log logs/$1.log