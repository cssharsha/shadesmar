#!/bin/bash
set -e

# Part 1: All setup before shell switch
sudo apt-get update
sudo apt-get install -y git-lfs htop tmux vim zsh

# Install clangd and bear first
sudo apt-get install -y \
    clangd \
    bear

# Create and write the script
sudo tee /usr/local/bin/gen-compile-commands << 'EOF'
#!/bin/bash
set -e  # Exit on any error

# Clean any existing compilation database
rm -f compile_commands.json

# Generate compilation database using bear
# First try a clean build to avoid potential issues
bazel clean
bear -- bazel build //... || {
    echo "Failed to generate compilation database"
    exit 1
}

# Verify the file exists and is not empty
if [ ! -s compile_commands.json ]; then
    echo "Error: compile_commands.json was not generated or is empty"
    exit 1
fi

# Only move if not already in workspace root
WORKSPACE_ROOT=$(git rev-parse --show-toplevel)
if [ "$PWD" != "$WORKSPACE_ROOT" ]; then
    mv compile_commands.json "$WORKSPACE_ROOT/"
fi
EOF

# Make the script executable
sudo chmod +x /usr/local/bin/gen-compile-commands

# Then continue with oh-my-zsh installation
sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended

# Download and install Dracula theme
git clone https://github.com/dracula/zsh.git ~/.oh-my-zsh/themes/dracula
ln -s ~/.oh-my-zsh/themes/dracula/dracula.zsh-theme ~/.oh-my-zsh/themes/dracula.zsh-theme

# Configure oh-my-zsh theme
sed -i 's/ZSH_THEME="robbyrussell"/ZSH_THEME="dracula"/' ~/.zshrc

# Install powerline fonts if using agnoster theme
sudo apt-get install -y fonts-powerline

# Install Go 1.21
curl -OL https://golang.org/dl/go1.21.8.linux-amd64.tar.gz
sudo rm -rf /usr/local/go
sudo tar -C /usr/local -xzf go1.21.8.linux-amd64.tar.gz
rm go1.21.8.linux-amd64.tar.gz

# Add Go to path (modify the existing PATH additions)
echo 'export PATH=/usr/local/go/bin:$PATH:$(go env GOPATH)/bin' >> ~/.zshrc
echo 'export PATH=/usr/local/go/bin:$PATH:$(go env GOPATH)/bin' >> ~/.bashrc

# Export the PATH immediately for the current session
export PATH=/usr/local/go/bin:$PATH

# Now continue with Bazelisk installation
/usr/local/go/bin/go install github.com/bazelbuild/bazelisk@latest
sudo ln -s $(go env GOPATH)/bin/bazelisk /usr/local/bin/bazel
sudo chmod 755 /usr/local/bin/bazel
sudo chmod 755 $(go env GOPATH)/bin/bazelisk

# Create a second script for remaining setup
cat > ~/.continue_setup.sh << 'EOF'
#!/bin/zsh
# Configure git
git config --global core.editor "code --wait"
git config --global pull.rebase false

# Set up Python tools
pip3 install --user \
    pre-commit \
    pylint \
    yapf

# Add local pip binaries to PATH (add to both bash and zsh)
echo 'export PATH=$PATH:$HOME/.local/bin' >> ~/.bashrc
echo 'export PATH=$PATH:$HOME/.local/bin' >> ~/.zshrc
export PATH=$PATH:$HOME/.local/bin

# Install clang tools
sudo apt-get install -y \
    clang-format \
    clang-tidy

# Create configuration files
echo "
BasedOnStyle: Google
IndentWidth: 4
ColumnLimit: 100
" > ~/.clang-format

# # Set up pre-commit hooks
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

chmod +x ~/.continue_setup.sh

# Switch to zsh and continue setup
sudo chsh -s $(which zsh) developer
echo "source ~/.continue_setup.sh" >> ~/.zshrc
exec zsh -l