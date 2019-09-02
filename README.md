# Modified ROS wrapper for pocketsphinx  

The following repositories were used in our wrapper, but are not required for installation:

* Original repository: https://github.com/mikeferguson/pocketsphinx  
* Also used repo: https://github.com/gorinars/ros_voice_control

It uses up-to-date pocketsphinx features and is independent of most external dependencies. Current repository is a ROS wrapper which incorporates those features. 

Additionally, code from http://blog.justsophie.com/python-speech-to-text-with-pocketsphinx/ was incorporated to modify the stream mode as follows:
* Sound is continually monitored, but only recorded when above a certain threshold.
* Sound is buffered for a certain time before recording begins, and prepended.
* Recording stops after a predefined period of silence.
* In this way, only phrases are processed, instead of a continuous stream.

## Installation 
1)   
    ```
    sudo apt-get install -y python python-dev python-pip build-essential swig libpulse-dev git
    sudo apt-get install python-pyaudio
    ```
2) Install the CMU SphinxBase library.
   First, clone sphinxbase into any convenient directory. The home directory (~/) works:
   ```
   cd ~
   git clone https://github.com/cmusphinx/sphinxbase.git
   ```
   Then, follow the instructions from https://github.com/cmusphinx/sphinxbase. By default, the library will be extracted to /usr/local/share/sphinxbase/

3) Install the CMU PocketSphinx library
   First, clone pocketsphinx into any convenient directory. The home directory (~/) works:
   ```
   cd ~
   git clone https://github.com/cmusphinx/pocketsphinx.git
   ```
   Then, follow the instructions from https://github.com/cmusphinx/pocketsphinx. By default, the library will be extracted to /usr/local/share/pocketsphinx/

4) Install the python pocketsphinx wrapper. (You will need to have pip preinstalled for this to work)
    ```
    sudo pip install pocketsphinx
    ```
5) Git clone this repository into the your catkin workspace's src folder:
  ```
  cd ~/catkin_ws/src
  git clone https://github.com/UTNuclearRoboticsPublic/pocketsphinx.git  
  ```
## Usage

### If you want to decode from a microphone stream:
1. In launch/pocketsphinx.launch set _stream:=True.
2. Then, launch:
```
roslaunch pocketsphinx pocketsphinx.launch
```

### If you want to decode from a wav file:
1. In launch/pocketsphinx.launch set _stream:=False.
2. Set _wavpath to the desired wav file path. An example path is given in launch/pocketsphinx.launch.
3. Then, launch:
``` 
roslaunch pocketsphinx pocketsphinx.launch
```

## Modifying the model / keywords / dictionary:

You can run this with any set of words. To do that, you need lexicon and keyword list files (check voice_cmd.dic and voice_cmd.kwlist for details). Then, create a custom .dict or .kwlist file and update config/pocketsphinx.yaml accordingly.

Word pronunciations for English can be found in CMUdict

You can also download pocketsphinx acoustic models for several other languages here

Read more about pocketsphinx on the official website: http://cmusphinx.sourceforge.net
