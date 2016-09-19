# Package installation
In your workspace/src directory run:
```sh
$ ln -s [path-to-cloned-repo]/1303/ZD/2/announcer announcer
$ ln -s [path-to-cloned-repo]/1303/ZD/2/listener listener
$ ln -s [path-to-cloned-repo]/1303/ZD/2/message message
```
# Execution
Run in separate terminals:
```sh
$ rosrun listener listener __name:Ivanov
$ rosrun listener listener __name:Petrov
$ rosrun listener listener __name:Sidorov
$ rosrun announcer announcer
```
