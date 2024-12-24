# What the heck is this?
This is modified repo from ROS2 Humble's tutorial of Simple Talker / Listener Python code. I added Makefile to make building and running node easier.


## Running the Talker Node

To run the `talker` node, use the following command:

```sh
make run NODE=talker
```

## Cleaning Builds

To clean the builds, use:

```sh
make clean
```

## Building the Project

To build the project again, use:

```sh
make build
```

## Fresh Build and Run

To perform a fresh build and then run the `talker` node, use:

```sh
make NODE=talker
```

This runs `clean`, `build`, and then `run` sequentially.