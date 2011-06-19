for f in $(find . -iname "*locate_choice.yaml.gz") ; do
    if [ -f $f ] ; then
        echo $f
        nf=${f:0:28}
        mv "$f" "${nf}.refine_choice.yaml.gz"
    fi
done

for f in $(find . -iname "locate.config.yaml") ; do
    if [ -f $f ] ; then
        echo $f
        mv "$f" "$(dirname $f)/refine.config.yaml"
    fi
done
