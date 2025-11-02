#!/usr/bin/env python3
"""
MASt3R Inference Service

This service runs MASt3R model inference and communicates with C++ code.
It can be run as:
1. A subprocess that reads/writes JSON via stdin/stdout
2. A gRPC server (optional, for more complex setups)
3. A ZeroMQ service (optional, for high-performance scenarios)

For simplicity, we start with stdin/stdout JSON communication.
"""

import sys
import json
import base64
import numpy as np
import torch
from pathlib import Path
from typing import Dict, List, Tuple, Optional

# MASt3R imports
try:
    from mast3r.model import AsymmetricMASt3R
    from mast3r.fast_nn import fast_reciprocal_NNs
    import mast3r.utils.path_to_dust3r
    from dust3r.inference import inference
    from dust3r.utils.image import load_images
    from PIL import Image
    import io
except ImportError as e:
    print(f"ERROR: MASt3R dependencies not found: {e}", file=sys.stderr)
    print("Please install MASt3R: pip install mast3r", file=sys.stderr)
    sys.exit(1)


class MASt3RInferenceService:
    def __init__(self, model_name: str = "naver/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric",
                 device: str = "cuda", image_size: int = 512):
        """
        Initialize MASt3R inference service

        Args:
            model_name: HuggingFace model name or path to checkpoint
            device: Device to run inference on ('cuda' or 'cpu')
            image_size: Input image resolution (default 512)
        """
        self.device = device if torch.cuda.is_available() else "cpu"
        self.image_size = image_size

        print(f"Loading MASt3R model: {model_name}", file=sys.stderr)
        print(f"Device: {self.device}", file=sys.stderr)

        # Load model
        try:
            self.model = AsymmetricMASt3R.from_pretrained(model_name).to(self.device)
            self.model.eval()
            print("Model loaded successfully", file=sys.stderr)
        except Exception as e:
            print(f"ERROR loading model: {e}", file=sys.stderr)
            raise

    def decode_image_from_base64(self, base64_string: str) -> np.ndarray:
        """Decode base64 encoded image to numpy array"""
        img_bytes = base64.b64decode(base64_string)
        img = Image.open(io.BytesIO(img_bytes))
        return np.array(img)

    def encode_image_to_base64(self, img_array: np.ndarray) -> str:
        """Encode numpy array to base64 string"""
        img = Image.fromarray(img_array)
        buffer = io.BytesIO()
        img.save(buffer, format='PNG')
        return base64.b64encode(buffer.getvalue()).decode('utf-8')

    def save_temp_images(self, img1_data: str, img2_data: str) -> Tuple[str, str]:
        """
        Save images to temporary files for loading

        Args:
            img1_data: Base64 encoded image 1
            img2_data: Base64 encoded image 2

        Returns:
            Tuple of temporary file paths
        """
        import tempfile

        img1 = self.decode_image_from_base64(img1_data)
        img2 = self.decode_image_from_base64(img2_data)

        # Create temporary files
        temp_dir = Path(tempfile.gettempdir()) / "mast3r_inference"
        temp_dir.mkdir(exist_ok=True)

        path1 = str(temp_dir / "image1.png")
        path2 = str(temp_dir / "image2.png")

        Image.fromarray(img1).save(path1)
        Image.fromarray(img2).save(path2)

        return path1, path2

    def run_inference(self, img1_path: str, img2_path: str,
                     subsample_factor: int = 8) -> Dict:
        """
        Run MASt3R inference on two images

        Args:
            img1_path: Path to first image
            img2_path: Path to second image
            subsample_factor: Subsampling factor for correspondences

        Returns:
            Dictionary with results including:
            - points_3d: Nx3 array of 3D points
            - pixels_prev: Nx2 array of pixel coordinates in image 1
            - pixels_curr: Nx2 array of pixel coordinates in image 2
            - descriptors_prev: NxD array of feature descriptors
            - descriptors_curr: NxD array of feature descriptors
            - confidence: N array of confidence scores
        """
        try:
            # Load images
            images = load_images([img1_path, img2_path], size=self.image_size)

            # Run inference
            with torch.no_grad():
                output = inference([tuple(images)], self.model, self.device, batch_size=1)

            # Extract results
            view1 = output['view1']
            view2 = output['view2']
            pred1 = output['pred1']
            pred2 = output['pred2']

            # Get descriptors
            desc1 = pred1['desc'].squeeze(0)  # Remove batch dimension
            desc2 = pred2['desc'].squeeze(0)

            # Get 3D points (pointmaps)
            pts3d_1 = pred1['pts3d'].squeeze(0) if 'pts3d' in pred1 else None
            pts3d_2 = pred2['pts3d'].squeeze(0) if 'pts3d' in pred2 else None

            # Find correspondences using fast reciprocal nearest neighbors
            matches_im1, matches_im2 = fast_reciprocal_NNs(
                desc1, desc2,
                subsample_or_initxy1=subsample_factor,
                device=self.device,
                dist='dot',
                block_size=2**13
            )

            # Convert to numpy
            matches_im1_np = matches_im1.cpu().numpy()  # Nx2 (x, y)
            matches_im2_np = matches_im2.cpu().numpy()  # Nx2 (x, y)

            # Extract 3D points at matched locations
            # MASt3R outputs dense pointmaps, we sample at correspondence locations
            if pts3d_1 is not None:
                # pts3d_1 shape: [H, W, 3]
                H, W = pts3d_1.shape[:2]

                # Convert pixel coordinates to indices
                y_indices_1 = np.clip(matches_im1_np[:, 1].astype(int), 0, H-1)
                x_indices_1 = np.clip(matches_im1_np[:, 0].astype(int), 0, W-1)

                # Sample 3D points
                pts3d_1_np = pts3d_1.cpu().numpy()
                points_3d = pts3d_1_np[y_indices_1, x_indices_1, :]  # Nx3
            else:
                # If no 3D points available, return zeros
                points_3d = np.zeros((len(matches_im1_np), 3))

            # Extract descriptor features at match locations for tracking
            # Descriptors shape: [C, H, W]
            desc1_np = desc1.cpu().numpy()
            desc2_np = desc2.cpu().numpy()

            C, H_desc, W_desc = desc1_np.shape

            # Scale match coordinates to descriptor resolution
            scale_x = W_desc / W
            scale_y = H_desc / H

            y_desc_1 = np.clip((matches_im1_np[:, 1] * scale_y).astype(int), 0, H_desc-1)
            x_desc_1 = np.clip((matches_im1_np[:, 0] * scale_x).astype(int), 0, W_desc-1)
            y_desc_2 = np.clip((matches_im2_np[:, 1] * scale_y).astype(int), 0, H_desc-1)
            x_desc_2 = np.clip((matches_im2_np[:, 0] * scale_x).astype(int), 0, W_desc-1)

            # Sample descriptors: [C, H, W] -> [N, C]
            descriptors_1 = desc1_np[:, y_desc_1, x_desc_1].T  # NxC
            descriptors_2 = desc2_np[:, y_desc_2, x_desc_2].T  # NxC

            # Compute confidence as descriptor similarity
            confidence = np.sum(descriptors_1 * descriptors_2, axis=1)  # Dot product
            confidence = (confidence + 1.0) / 2.0  # Normalize to [0, 1]

            return {
                'success': True,
                'num_points': len(points_3d),
                'points_3d': points_3d.tolist(),
                'pixels_prev': matches_im1_np.tolist(),
                'pixels_curr': matches_im2_np.tolist(),
                'descriptors_prev': descriptors_1.tolist(),
                'descriptors_curr': descriptors_2.tolist(),
                'confidence': confidence.tolist(),
            }

        except Exception as e:
            print(f"ERROR during inference: {e}", file=sys.stderr)
            import traceback
            traceback.print_exc(file=sys.stderr)
            return {
                'success': False,
                'error': str(e)
            }

    def handle_request(self, request: Dict) -> Dict:
        """
        Handle a single inference request

        Request format:
        {
            "command": "inference",
            "image1": "<base64_encoded_image>",
            "image2": "<base64_encoded_image>",
            "subsample_factor": 8
        }

        Returns:
        {
            "success": True/False,
            "points_3d": [[x, y, z], ...],
            "pixels_prev": [[x, y], ...],
            "pixels_curr": [[x, y], ...],
            ...
        }
        """
        command = request.get('command', 'inference')

        if command == 'inference':
            # Save images to temporary files
            img1_path, img2_path = self.save_temp_images(
                request['image1'],
                request['image2']
            )

            subsample = request.get('subsample_factor', 8)

            # Run inference
            result = self.run_inference(img1_path, img2_path, subsample)

            # Clean up temp files
            Path(img1_path).unlink(missing_ok=True)
            Path(img2_path).unlink(missing_ok=True)

            return result

        elif command == 'ping':
            return {'success': True, 'message': 'pong'}

        elif command == 'shutdown':
            return {'success': True, 'message': 'shutting down'}

        else:
            return {'success': False, 'error': f'Unknown command: {command}'}

    def run_service(self):
        """
        Run the service loop, reading requests from stdin and writing responses to stdout
        """
        print("MASt3R Inference Service started", file=sys.stderr)
        print("Waiting for requests on stdin...", file=sys.stderr)

        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue

            try:
                request = json.loads(line)
                response = self.handle_request(request)

                # Write response to stdout as JSON
                print(json.dumps(response), flush=True)

                # Check for shutdown
                if request.get('command') == 'shutdown':
                    break

            except Exception as e:
                error_response = {
                    'success': False,
                    'error': f'Request processing failed: {str(e)}'
                }
                print(json.dumps(error_response), flush=True)


def main():
    import argparse

    parser = argparse.ArgumentParser(description='MASt3R Inference Service')
    parser.add_argument('--model', default='naver/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric',
                       help='Model name or path')
    parser.add_argument('--device', default='cuda', choices=['cuda', 'cpu'],
                       help='Device to run inference on')
    parser.add_argument('--image-size', type=int, default=512,
                       help='Input image size')

    args = parser.parse_args()

    # Create and run service
    service = MASt3RInferenceService(
        model_name=args.model,
        device=args.device,
        image_size=args.image_size
    )

    service.run_service()


if __name__ == '__main__':
    main()
