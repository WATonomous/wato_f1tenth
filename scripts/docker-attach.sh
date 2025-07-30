#!/bin/bash
set -e

docker-attach () {
    container_names=($(docker ps --format "{{.Names}}"))

    echo "available list of containers to attach to :"
    for i in "${!container_names[@]}"; do
        echo "$i. ${container_names[$i]}"
    done

    read -p 'select : ' index

    clear

    if [[ $index -lt 0 || $index -ge ${#container_names[@]} ]]; then 
        echo "select valid container"
    else 
        echo "attaching to ${container_names[$index]}"
        docker exec -it ${container_names[$index]} bash
    fi

}