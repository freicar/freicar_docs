# Ubuntu 20.x installation on FreiCAR PC
Description of the initial setup for the FreiCAR PCs. As a course participant, please ignore this page. The cars will already be set up for you.

## Requirements
- Assembled and powered car PC
-  8 GB flash drive

## Procedure
- Follow https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview
    - In step 9 Create Your Login Details :
        - Your name: **freicar**
        - Your computer's name: **freicarX** where **X** is the car number, chosen based on the rf joystick emitter-receiver.
        - User name: **freicar**
        - Password: **freicar**

# Libraries and Packages

## Update CA certificates

- `sudo apt-get install --reinstall ca-certificates`

- `sudo mkdir /usr/local/share/ca-certificates/cacert.org`

- `sudo wget -P /usr/local/share/ca-certificates/cacert.org http://www.cacert.org/certs/root.crt http://www.cacert.org/certs/class3.crt`

- `sudo update-ca-certificates`

## Git
- `sudo apt install git`
- `git config --global http.sslCAinfo /etc/ssl/certs/ca-certificates.crt`

## Terminator
- `sudo apt install terminator`

## Devices drivers

- After following the [FreiCAR installation](../environment/), go to the directory where you cloned the `docker` repository, then to `freicar_ws/src/drivers/udevs` and run `./copy_to_system.sh`. Use `chmod +x` on the file if necessary.

## Changes to docker

- `fcc` into docker, then go to `~/freicar_ws/src/freicar_drivers/zed_factory_calibrations` and run `./copy_zed_calibrations.sh`.
- In a system terminal, go to the `docker` directory and run `./commit_changes.bash`

```
