#!/bin/bash
set -e

# Part 1: All setup before shell switch
sudo apt-get update
sudo apt-get install -y git-lfs htop tmux vim zsh

# Install oh-my-zsh
sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended

# Download and install Dracula theme
git clone https://github.com/dracula/zsh.git ~/.oh-my-zsh/themes/dracula
ln -s ~/.oh-my-zsh/themes/dracula/dracula.zsh-theme ~/.oh-my-zsh/themes/dracula.zsh-theme

# Configure oh-my-zsh theme
sed -i 's/ZSH_THEME="robbyrussell"/ZSH_THEME="dracula"/' ~/.zshrc

# Install powerline fonts
sudo apt-get install -y fonts-powerline

# Install Go 1.21
curl -OL https://golang.org/dl/go1.21.8.linux-amd64.tar.gz
sudo rm -rf /usr/local/go
sudo tar -C /usr/local -xzf go1.21.8.linux-amd64.tar.gz
rm go1.21.8.linux-amd64.tar.gz

# Add Go to path
echo 'export PATH=/usr/local/go/bin:$PATH:$(go env GOPATH)/bin' >> ~/.zshrc
echo 'export PATH=/usr/local/go/bin:$PATH:$(go env GOPATH)/bin' >> ~/.bashrc

# Export the PATH immediately
export PATH=/usr/local/go/bin:$PATH

# Install Bazelisk
/usr/local/go/bin/go install github.com/bazelbuild/bazelisk@latest
GOPATH="$(go env GOPATH)"
sudo ln -s "${GOPATH}/bin/bazelisk" /usr/local/bin/bazel
sudo chmod 755 /usr/local/bin/bazel
sudo chmod 755 "${GOPATH}/bin/bazelisk"

# Install Python tools
pip3 install --user \
    pre-commit \
    pylint \
    yapf

# Add local pip binaries to PATH
echo 'export PATH=$PATH:$HOME/.local/bin' >> ~/.bashrc
echo 'export PATH=$PATH:$HOME/.local/bin' >> ~/.zshrc

# Install clang tools
sudo apt-get install -y \
    clang-format \
    clang-tidy \
    clangd \
    bear

# Create clang-format config
echo "
BasedOnStyle: Google
IndentWidth: 4
ColumnLimit: 100
" > ~/.clang-format

# Create pre-commit config
cat > .pre-commit-config.yaml << 'EOF'
repos:
-   repo: https://github.com/pre-commit/pre-commit-hooks
    rev: v4.4.0
    hooks:
    -   id: trailing-whitespace
    -   id: end-of-file-fixer
    -   id: check-yaml
    -   id: check-added-large-files
EOF

# Final setup
echo "source ~/.cargo/env" >> ~/.zshrc
sudo chsh -s "$(which zsh)" developer
