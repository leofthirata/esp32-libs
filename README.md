## Setup
1. Open folder in Visual Studio Code.
2. Download and install [Docker](https://docs.docker.com/desktop/setup/install/linux/). The instructions mentioned in the Docker article will probably lead you to errors like docker-cli and docker-desktop not found. So follow these:
```bash
# Set the docker repository
$ sudo apt install -y ca-certificates curl gnupg lsb-release
$ sudo mkdir -p /etc/apt/keyrings
$ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Update all packages
$ sudo apt update -y

# Install docker and fix any broken stuff
$ sudo apt install ./docker-desktop-amd64.deb
$ sudo apt --fix-broken install (if needed)

# Run docker
$ systemctl --user start docker-desktop

# Give user permissions to access USB devices
$ sudo usermod -a -G dialout $USER

# If the error below shows up
docker: Cannot connect to the Docker daemon at unix:///var/run/docker.sock. Is the docker daemon running?.
See 'docker run --help'.
Run
$ sudo systemctl stop docker
$ sudo systemctl enable docker.service
$ sudo systemctl start docker.service

Or the docker.sock path is something like ~/.docker/desktop/docker.sock
$ sudo ln -sv /var/run/docker.sock ~/.docker/desktop/docker.sock
```
4. Install Dev Container extension for Visual Studio Code.
5. Reopen folder in Container.
6. Ta-dah!
