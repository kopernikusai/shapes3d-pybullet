cmd_line="$@"

echo "Executing in the docker DEV container:"
echo $cmd_line

docker run \
    -ti --rm \
    --gpus=all \
    --env="DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd):/shapes3d:rw" \
    --entrypoint="/empty_entrypoint.sh" \
    shapes3d $cmd_line
