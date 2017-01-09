#! /bin/bash
#docker run -it \
#	--env="DISPLAY=192.168.99.1:0" \
#	--volume "$HOME/sharefolder:/sharefolder" \
#	html /bin/bash \

docker run --rm -e DISPLAY=192.168.0.108:0 \
    -i -t \
    -v /Volumes/backup:/home/backup \
    -v /Users/jmlbeaujour/Documents:/home/Documents \
  	sdc-u14 #/bin/bash \