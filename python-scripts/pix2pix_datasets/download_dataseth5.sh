echo "Checking for opencv"
if python -c "import cv2" ; then
  echo "opencv installed"
else
  echo "opencv required"
fi

FILE=$1

if [[ $FILE != "cityscapes" &&  $FILE != "night2day" &&  $FILE != "edges2handbags" && $FILE != "edges2shoes" && $FILE != "facades" && $FILE != "maps" ]]; then
  echo "Available datasets are cityscapes, night2day, edges2handbags, edges2shoes, facades, maps"
  exit 1
fi

echo "Specified [$FILE]"

if [[$FILE == "cityscapes"]];  then
IN_CHANNELS = 3
OUT_CHANNELS = 3
fi

if [[$FILE == "night2day"]];  then
IN_CHANNELS = 3
OUT_CHANNELS = 3
fi

if [[$FILE == "edges2handbags"]];  then
IN_CHANNELS = 1
OUT_CHANNELS = 3
fi

if [[$FILE == "edges2shoes"]];  then
IN_CHANNELS = 1
OUT_CHANNELS = 3
fi

if [[$FILE == "facades"]];  then
IN_CHANNELS = 3
OUT_CHANNELS = 3
fi

if [[$FILE == "maps"]];  then
IN_CHANNELS = 3
OUT_CHANNELS = 3
fi

URL=http://efrosgans.eecs.berkeley.edu/pix2pix/datasets/$FILE.tar.gz
TAR_FILE=./datasets/$FILE.tar.gz
TARGET_DIR=./datasets/$FILE/
wget -N $URL -O $TAR_FILE
mkdir -p $TARGET_DIR
tar -zxvf $TAR_FILE -C ./datasets/
rm $TAR_FILE

python3 datasets2h5.py -p $TARGET_DIR -i $IN_CHANNELS -o $OUT_CHANNELS

rm -rf ./$TARGET_DIR/test
rm -rf ./$TARGET_DIR/train
rm -rf ./$TARGET_DIR/val