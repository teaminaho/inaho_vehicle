# Setup

## Mbed
### Prepare firmware

- The firmware code is in [Mbed | inaho_vehicle](https://os.mbed.com/teams/inaho/code/inaho_vehicle/).
- You can build and download the firmware on Mbed online compiler.
- See the repository page for details.

### Set udev rule

Set udev rules to access USB serial with a fixed path (`/dev/ttyMbed`).

```shell
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0d28", ATTRS{idProduct}=="0204", SYMLINK+="ttyMbed", MODE="0666"' | sudo tee /etc/udev/rules.d/48-mbed.rules
sudo udevadm control --reload && sudo udevadm trigger
```

## Docker

### Install Docker & Docker Compose

Here is an example.
Please change appropriately depending on the OS and docker version.

```shell
sudo apt-get update
sudo apt-get install -y apt-transport-https ca-certificates \
    curl gnupg-agent software-properties-common

curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) stable"
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io

sudo curl -L "https://github.com/docker/compose/releases/download/1.29.2/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose
sudo systemctl enable docker.service
```

### Enable experimental features
``` shell
echo '{"experimental": true}' | sudo tee /etc/docker/daemon.json
sudo systemctl restart docker.service
```

### Enable docker without sudo
``` shell
sudo groupadd docker
sudo gpasswd -a $USER docker
sudo systemctl restart docker
```
