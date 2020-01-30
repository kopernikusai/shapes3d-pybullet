cmd_line="$@"

echo "Executing in the docker container:"
echo $cmd_line

docker run \
    --rm -ti \
    --gpus=all \
    --env="DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    shapes3d $cmd_line
