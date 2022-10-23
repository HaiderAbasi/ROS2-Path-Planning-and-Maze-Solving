# Docker Commands


## IMAGES
- Downloading Images
```
docker pull <image name>
```
- List all images
```
docker images -a
```
- Delete single Image
```
docker rmi <image id>
```
- All images delete
```
docker rmi $(docker images  -q)
```
- Dangling images delete
```
docker rmi $(docker images --filter "dangling=true" -q --no-trunc)
```



## CONTAINERS
- Creating a interactive container  from image
```
docker run -it <image name>
```
- Giving Name to a container while creating
```
docker run --name <container name> <image name>
```
- Start a stopped Container
```
docker start (container_id)
```
- Stop all containers
```
sudo docker kill $(sudo docker ps -a)
```
- Connect shell to running container
```
docker exec -it (container_id) bash
```
- Delete single Container
```
docker rm <container id or container name>
```
- Delete all containers
```
docker rm $(docker ps -a -q)
```
## Building Image from Docker File
- Terminal from same directory
```
docker built -t <image name > .
```

## GUI arguments
### Windows

## Complete Examples
- Running a container with GUI enabled for Windows
```
docker run -it --name r2_pathplanning_container -e DISPLAY=host.docker.internal:0.0 -e haiderabbasi333/ros2-pathplanning-course:1 bash

```

## Mistakes
- Mixing options
```
docker run --name -it <container name> <image name>
```
here you should have provided name for **--name** before giving another option **-it**

