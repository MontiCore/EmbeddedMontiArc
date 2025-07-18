for Line in $(cat $1)
do
    # echo "Line: $Line"
    words=($(echo $Line | tr "=" "\n"))
    # echo "First word: ${words[0]}, Second: ${words[1]}"
    declare "${words[0]}=${words[1]}"
done