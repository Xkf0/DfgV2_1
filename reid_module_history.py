# reid_module.py
import torch
import torch.nn as nn
import torchvision.models as models
import torchvision.transforms as transforms
from PIL import Image
import cv2
import numpy as np
from vision_utils import get_rotated_rect_crop
# 移除 from config import SIMILARITY_THRESHOLD

class FeatureExtractor:
    def __init__(self, device='cuda' if torch.cuda.is_available() else 'cpu'):
        self.device = torch.device(device)
        print(f"FeatureExtractor 使用设备: {self.device}")
        
        self.model = models.resnet18(pretrained=True)
        self.model = nn.Sequential(*list(self.model.children())[:-1])
        self.model = self.model.to(self.device)
        self.model.eval()

        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

    def extract_features(self, image):
        try:
            if len(image.shape) == 3:
                pil_image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
            else:
                pil_image = Image.fromarray(image).convert('RGB')
            
            input_tensor = self.transform(pil_image).unsqueeze(0).to(self.device)
            with torch.no_grad():
                features = self.model(input_tensor)
                features = features.cpu().flatten().numpy()
            return features
        except Exception as e:
            print(f"特征提取失败: {e}")
            return None

    def cosine_similarity(self, feat1, feat2):
        if feat1 is None or feat2 is None:
            return 0.0
        return np.dot(feat1, feat2) / (np.linalg.norm(feat1) * np.linalg.norm(feat2) + 1e-8)

def extract_mask_crop_with_features(image, contour, feature_extractor):
    crop = get_rotated_rect_crop(image, contour)
    if crop is None:
        return None, None
    features = feature_extractor.extract_features(crop)
    return crop, features

# 修改：增加 cfg 参数
def filter_objects_by_similarity(current_objects, warped_frame, feature_matching_enabled, 
                               reference_features, feature_extractor, cfg):
    if not feature_matching_enabled or reference_features is None:
        return current_objects
    
    filtered = []
    for obj in current_objects:
        crop, features = extract_mask_crop_with_features(warped_frame, obj['contour'], feature_extractor)
        if features is not None:
            similarity = feature_extractor.cosine_similarity(reference_features, features)
            # 使用 cfg.similarity_threshold
            if similarity >= cfg.similarity_threshold:
                filtered.append(obj)
            else:
                pass # Filtered out
        else:
            filtered.append(obj)
    return filtered