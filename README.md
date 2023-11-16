# cyber-android
This a repo for the challenge of the afternoon

For this challenge we will use the code in a separated repo

## Get serial_megapi and install it
```
cd ~/workspace
git clone https://github.com/flochre/serial_megapi.git
cd serial_megapi
mkdir build && cd build/
cmake ..
make
sudo make install
sudo ldconfig
```

## Use this repo to create our code
```
cd ~/workspace
# clone your newly created repo
```

## write you code and compile it

You should now write create a folder with your robot name
inside create a c-file like cyberAndroid.c

now you can compile your code

```
gcc -o cyberAndroid cyberAndroid.c -lserial_megapi -lwiringPi
```
