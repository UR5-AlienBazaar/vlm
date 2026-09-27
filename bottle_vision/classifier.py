"""DINOv2 classification head shared by training and live inference."""
import torch
from torch import nn


class DinoBottleClassifier(nn.Module):
    def __init__(self, model_id: str, classes: list[str], dropout: float = .2):
        super().__init__()
        from transformers import AutoModel
        self.backbone = AutoModel.from_pretrained(model_id)
        width = self.backbone.config.hidden_size
        self.classes = classes
        self.head = nn.Sequential(nn.Linear(width, width), nn.GELU(), nn.Dropout(dropout), nn.Linear(width, len(classes)))

    def set_stage(self, finetune: bool) -> None:
        for parameter in self.backbone.parameters():
            parameter.requires_grad = False
        if finetune:
            layers = getattr(getattr(self.backbone, "encoder", None), "layer", [])
            for layer in layers[-4:]:
                for parameter in layer.parameters():
                    parameter.requires_grad = True

    def forward(self, pixel_values):
        return self.head(self.backbone(pixel_values=pixel_values).pooler_output)
