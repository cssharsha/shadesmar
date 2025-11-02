#!/bin/bash
# Setup script for MASt3R integration

set -e

echo "========================================="
echo "MASt3R Integration Setup"
echo "========================================="

# Check if running in Docker
if [ -f /.dockerenv ]; then
    echo "✓ Running inside Docker container"
else
    echo "⚠ Not running in Docker - this script is designed for docker-dev-1 container"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Check Python version
echo ""
echo "Checking Python installation..."
PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
echo "Python version: $PYTHON_VERSION"

if ! python3 -c "import sys; sys.exit(0 if sys.version_info >= (3, 8) else 1)"; then
    echo "❌ Python 3.8+ required"
    exit 1
fi
echo "✓ Python version OK"

# Check CUDA availability
echo ""
echo "Checking CUDA availability..."
if python3 -c "import torch; print(torch.cuda.is_available())" 2>/dev/null | grep -q "True"; then
    echo "✓ CUDA is available"
    DEVICE="cuda"
else
    echo "⚠ CUDA not available - will use CPU (slower)"
    DEVICE="cpu"
fi

# Install MASt3R dependencies
echo ""
echo "Installing MASt3R dependencies..."

# Check if MASt3R is already installed
if python3 -c "import mast3r" 2>/dev/null; then
    echo "✓ MASt3R already installed"
else
    echo "Installing MASt3R from GitHub..."

    # Try pip install first
    if pip install git+https://github.com/naver/mast3r.git; then
        echo "✓ MASt3R installed successfully via pip"
    else
        echo "❌ Failed to install MASt3R via pip"
        echo "Try manual installation:"
        echo "  git clone https://github.com/naver/mast3r.git"
        echo "  cd mast3r && pip install -e ."
        exit 1
    fi
fi

# Verify installation
echo ""
echo "Verifying MASt3R installation..."
python3 -c "
import mast3r
import mast3r.utils.path_to_dust3r
from dust3r.inference import inference
from mast3r.model import AsymmetricMASt3R
print('✓ All MASt3R imports successful')
"

if [ $? -ne 0 ]; then
    echo "❌ MASt3R verification failed"
    exit 1
fi

# Test the inference service
echo ""
echo "Testing Python inference service..."
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo "Script directory: $SCRIPT_DIR"

# Make the Python script executable
chmod +x "$SCRIPT_DIR/mast3r_inference_service.py"

# Test with ping command
echo '{"command": "ping"}' | timeout 30 python3 "$SCRIPT_DIR/mast3r_inference_service.py" --device "$DEVICE" 2>&1 &
PID=$!

# Wait a bit for model loading
sleep 3

# Check if process is still running
if ps -p $PID > /dev/null 2>&1; then
    echo "✓ Python inference service started successfully"
    kill $PID 2>/dev/null || true
    wait $PID 2>/dev/null || true
else
    echo "⚠ Python service may have issues - check logs"
fi

# Download model weights (optional but recommended)
echo ""
echo "Downloading model weights..."
python3 -c "
from mast3r.model import AsymmetricMASt3R
import torch

model_name = 'naver/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric'
print(f'Loading model: {model_name}')
device = '$DEVICE'
model = AsymmetricMASt3R.from_pretrained(model_name).to(device)
print(f'✓ Model loaded successfully on {device}')
print(f'  Model parameters: {sum(p.numel() for p in model.parameters())/1e6:.1f}M')
"

if [ $? -eq 0 ]; then
    echo "✓ Model weights downloaded and cached"
else
    echo "❌ Failed to download model weights"
    exit 1
fi

# Build tracking library
echo ""
echo "Building tracking library with MASt3R..."
cd /home/sree/Matrix/shadesmar
if bazel build //tracking:tracking; then
    echo "✓ Tracking library built successfully"
else
    echo "❌ Build failed"
    exit 1
fi

# Summary
echo ""
echo "========================================="
echo "Setup Complete!"
echo "========================================="
echo ""
echo "Next steps:"
echo "1. Edit core/graph/include/core/graph/graph_adapter.hpp to include MASt3R tracker"
echo "2. Replace orb_tracker_ with mast3r_tracker_ (or use both)"
echo "3. Rebuild the project: bazel build //..."
echo ""
echo "See tracking/2d/MAST3R_INTEGRATION.md for detailed integration guide"
echo ""
echo "Device: $DEVICE"
echo "Python: $PYTHON_VERSION"
echo ""
