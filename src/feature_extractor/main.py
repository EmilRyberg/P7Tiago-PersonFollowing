import torch
import numpy as np
from model import Block35

if __name__ == "__main__":
    test_tensor = torch.randn((1, 3, 299, 299))
    b35 = Block35(3)
    b35.eval()
    result = b35(test_tensor)
    print(result.shape)
    print(result)
