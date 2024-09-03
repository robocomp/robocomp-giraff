# speech
Intro to component here

## Dependencies
```bash
sudo apt-get install sox libsox-fmt-all
```

## Audio problem
You should select the audio device 

```bash
sudo apt install pulseaudio-utils

pactl list short sinks #devices list
pactl set-default-sink <DEVICE_ID>
```


## Configuration parameters
As any other component, *speech* needs a configuration file to start. In
```
etc/config
```
you can find an example of a configuration file. We can find there the following lines:
```
EXAMPLE HERE
```

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd <speech's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/speech config
```
