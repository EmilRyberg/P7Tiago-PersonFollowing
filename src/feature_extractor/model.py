import torch.nn.functional as F
import torch.nn as nn
import torch
import numpy as np
from typing import Union, Tuple, Optional


class Block35(nn.Module):
    def __init__(self, input_channels: int, output_channels=256, scale=1.0, activation_fn=nn.ReLU(), scope=None, reuse=None):
        super(Block35, self).__init__()
        self.scale = scale
        self.activation_fn = activation_fn
        self.scope = scope
        self.reuse = reuse
        self.branch1 = ConvBlock(input_channels, 32, kernel_size=1, activation=activation_fn)
        self.branch2 = nn.ModuleList([
            ConvBlock(input_channels, 32, kernel_size=1, activation=activation_fn),
            ConvBlock(32, 32, kernel_size=3, padding=1, activation=activation_fn)
        ])
        self.branch3 = nn.ModuleList([
            ConvBlock(input_channels, 32, kernel_size=1, activation=activation_fn),
            ConvBlock(32, 32, kernel_size=3, padding=1, activation=activation_fn),
            ConvBlock(32, 32, kernel_size=3, padding=1, activation=activation_fn)
        ])
        self.final = ConvBlock(32 * 3, output_channels, kernel_size=1, batch_norm=False, activation=None)

    def forward(self, x):
        branch1_out = self.branch1(x)
        branch2_out = x
        for m in self.branch2:
            branch2_out = m(branch2_out)
        branch3_out = x
        for m in self.branch3:
            branch3_out = m(branch3_out)
        concat = torch.cat([branch1_out, branch2_out, branch3_out], 1)
        x = self.final(concat)
        x = x * self.scale
        if self.activation_fn is not None:
            x = self.activation_fn(x)
        return x


class ConvBlock(nn.Module):
    def __init__(self, channels_in: int, channels_out: int, kernel_size: Union[int, Tuple[int, int]],
                 padding: Union[int, Tuple[int, int]] = 0, batch_norm: bool = True,
                 activation: Optional[object] = nn.ReLU(),
                 stride: int = 1):
        super(ConvBlock, self).__init__()
        self.conv = nn.Conv2d(in_channels=channels_in,
                      out_channels=channels_out,
                      kernel_size=kernel_size, bias=not batch_norm, padding=padding, stride=stride)
        self.has_batch_norm = False
        self.batch_norm = None

        if batch_norm:
            self.batch_norm = nn.BatchNorm2d(channels_out)
            self.has_batch_norm = True

        self.activation = activation

    def forward(self, x):
        x = self.conv(x)
        if self.has_batch_norm:
            x = self.batch_norm(x)
        if self.activation is not None:
            x = self.activation(x)
        return x


def calculate_padding(kernel_size):
    return (kernel_size - 1) // 2


