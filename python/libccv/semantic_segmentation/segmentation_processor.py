import torch
import torchvision.transforms as T
from PIL import Image
import numpy as np
import cv2
from libccv.common.dataset import PASCAL_VOC
from libccv.common.camera_utils import CameraCapture


class SegmentationModelLoader:
    """Unified model loader for segmentation models."""

    @staticmethod
    def load_pretrained(model_name, num_classes=None):
        """Load pre-trained segmentation models with consistent interface.

        Args:
            model_name (str): Name of the model to load
            num_classes (int, optional): Number of classes for the model

        Returns:
            torch.nn.Module: Loaded model
        """
        import torchvision.models as models

        # Model mapping
        model_map = {
            "fcn_resnet101": models.segmentation.fcn_resnet101,
            "deeplabv3_resnet50": models.segmentation.deeplabv3_resnet50,
            "deeplabv3_resnet101": models.segmentation.deeplabv3_resnet101,
            "deeplabv3_mobilenet_v3_large": models.segmentation.deeplabv3_mobilenet_v3_large,
        }

        if model_name not in model_map:
            raise ValueError(f"Unsupported model: {model_name}")

        # Load model with appropriate weights
        weights_map = {
            "fcn_resnet101": models.segmentation.FCN_ResNet101_Weights.COCO_WITH_VOC_LABELS_V1,
            "deeplabv3_resnet50": models.segmentation.DeepLabV3_ResNet50_Weights.COCO_WITH_VOC_LABELS_V1,
            "deeplabv3_resnet101": models.segmentation.DeepLabV3_ResNet101_Weights.COCO_WITH_VOC_LABELS_V1,
            "deeplabv3_mobilenet_v3_large": models.segmentation.DeepLabV3_MobileNet_V3_Large_Weights.COCO_WITH_VOC_LABELS_V1,
        }

        model = model_map[model_name](weights=weights_map[model_name])
        model.eval()

        # Modify final layer if num_classes is specified
        if num_classes is not None:
            if hasattr(model, "classifier") and hasattr(model.classifier, "4"):
                in_channels = model.classifier[4].in_channels
                model.classifier[4] = torch.nn.Conv2d(
                    in_channels, num_classes, kernel_size=(1, 1), stride=(1, 1)
                )
            elif hasattr(model, "classifier") and hasattr(model.classifier, "3"):
                in_channels = model.classifier[3].in_channels
                model.classifier[3] = torch.nn.Conv2d(
                    in_channels, num_classes, kernel_size=(1, 1), stride=(1, 1)
                )

        return model

    @staticmethod
    def load_custom(model_name, model_path, num_classes):
        """Load custom trained segmentation models.

        Args:
            model_name (str): Base model architecture
            model_path (str): Path to the trained model weights
            num_classes (int): Number of classes for the model

        Returns:
            torch.nn.Module: Loaded model with custom weights
        """
        import torchvision.models as models

        # Load base model
        model_map = {
            "deeplabv3_resnet50": models.segmentation.deeplabv3_resnet50,
        }

        if model_name not in model_map:
            raise ValueError(f"Unsupported model: {model_name}")

        model = model_map[model_name](pretrained=True)

        # Modify final layer
        if hasattr(model, "classifier") and hasattr(model.classifier, "4"):
            in_channels = model.classifier[4].in_channels
            model.classifier[4] = torch.nn.Conv2d(
                in_channels, num_classes, kernel_size=(1, 1), stride=(1, 1)
            )

        # Load custom weights
        model.load_state_dict(torch.load(model_path))
        model.eval()

        return model


class SegmentationTransforms:
    """Unified transformation pipeline for segmentation tasks."""

    @staticmethod
    def get_preprocessing_transform(size=None, normalize=True):
        """Get preprocessing transformation pipeline.

        Args:
            size (tuple, optional): Target size (height, width)
            normalize (bool): Whether to apply normalization

        Returns:
            torchvision.transforms.Compose: Transformation pipeline
        """
        transforms_list = [T.ToPILImage()]

        if size is not None:
            transforms_list.append(T.Resize(size))

        transforms_list.append(T.ToTensor())

        if normalize:
            transforms_list.append(
                T.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225))
            )

        return T.Compose(transforms_list)

    @staticmethod
    def get_annotation_transform(size=None):
        """Get annotation transformation pipeline.

        Args:
            size (tuple, optional): Target size (height, width)

        Returns:
            torchvision.transforms.Compose: Transformation pipeline
        """
        transforms_list = [T.ToPILImage()]

        if size is not None:
            transforms_list.append(T.Resize(size))

        transforms_list.append(T.ToTensor())

        return T.Compose(transforms_list)


class SegmentationProcessor:
    """Unified processor for segmentation tasks."""

    def __init__(self, model, device=None):
        """Initialize segmentation processor.

        Args:
            model (torch.nn.Module): Segmentation model
            device (torch.device, optional): Device to run inference on
        """
        self.model = model
        self.device = device or torch.device(
            "cuda" if torch.cuda.is_available() else "cpu"
        )
        self.model = self.model.to(self.device)

    def preprocess(self, image, target_size=None):
        """Preprocess image for segmentation.

        Args:
            image (np.ndarray): Input image
            target_size (tuple, optional): Target size (height, width)

        Returns:
            torch.Tensor: Preprocessed image tensor
        """
        transform = SegmentationTransforms.get_preprocessing_transform(target_size)
        inp = transform(image).unsqueeze(0)
        return inp.to(self.device)

    def infer(self, image_tensor):
        """Run inference on preprocessed image.

        Args:
            image_tensor (torch.Tensor): Preprocessed image tensor

        Returns:
            torch.Tensor: Model output
        """
        with torch.no_grad():
            output = self.model(image_tensor)
            if isinstance(output, dict) and "out" in output:
                return output["out"]
            return output

    def postprocess(self, output, original_size):
        """Postprocess model output.

        Args:
            output (torch.Tensor): Model output
            original_size (tuple): Original image size (height, width)

        Returns:
            np.ndarray: Segmentation map
        """
        # Resize to original size if needed
        if output.shape[-2:] != original_size:
            resize_transform = T.Resize(original_size)
            output = resize_transform(output)

        # Get prediction classes
        if output.dim() == 4:  # Batch dimension present
            seg = torch.argmax(output.squeeze(), dim=0).cpu().numpy()
        else:
            seg = torch.argmax(output, dim=0).cpu().numpy()

        return seg

    def visualize(self, image, segmentation, mode="overlay", alpha=0.5):
        """Visualize segmentation results.

        Args:
            image (np.ndarray): Original image
            segmentation (np.ndarray): Segmentation map
            mode (str): Visualization mode ("overlay", "side_by_side", "mask_only")
            alpha (float): Transparency for overlay mode

        Returns:
            np.ndarray: Visualization result
        """
        # Decode segmentation map
        seg_rgb = PASCAL_VOC.decode_segmap(segmentation)

        if mode == "overlay":
            # Overlay segmentation on original image
            result = cv2.addWeighted(image, 1 - alpha, seg_rgb, alpha, 0)
            return result
        elif mode == "side_by_side":
            # Show original and segmentation side by side
            return np.hstack([image, seg_rgb])
        elif mode == "mask_only":
            # Show only segmentation mask
            return seg_rgb
        else:
            raise ValueError(f"Unsupported visualization mode: {mode}")


class VideoSegmentation(SegmentationProcessor):
    """Video segmentation processor with camera integration."""

    def __init__(self, model, camera_id=0, device=None):
        """Initialize video segmentation processor.

        Args:
            model (torch.nn.Module): Segmentation model
            camera_id (int): Camera ID for video capture
            device (torch.device, optional): Device to run inference on
        """
        super().__init__(model, device)
        self.camera = CameraCapture(camera_id)

    def run_realtime_segmentation(self, mode="overlay", alpha=0.5):
        """Run real-time segmentation on video stream.

        Args:
            mode (str): Visualization mode
            alpha (float): Transparency for overlay mode
        """
        try:
            while True:
                ret, frame = self.camera.read()
                if not ret:
                    continue

                # Preprocess frame
                original_size = (frame.shape[0], frame.shape[1])
                input_tensor = self.preprocess(frame)

                # Run inference
                output = self.infer(input_tensor)

                # Postprocess
                seg_map = self.postprocess(output, original_size)

                # Visualize
                result = self.visualize(frame, seg_map, mode, alpha)

                # Display result
                cv2.imshow("Segmentation", result)

                # Break on 'q' key press
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        finally:
            self.camera.release()
            cv2.destroyAllWindows()
