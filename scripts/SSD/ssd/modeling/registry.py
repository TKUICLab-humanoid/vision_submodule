from ssd.utils.registry import Registry

BACKBONES = Registry()       
BOX_HEADS = Registry()
BOX_PREDICTORS = Registry()

# registry的主要作用是作為模型字典來保存生成的不同網絡結構模型，
# 其有兩處定義分別為utils裡面定義的Registry.py和在modeling裡面定義的Registry.py。
# 前者是定義數據類型，後者是這個字典類型的一個集合，為保存不同的網絡結構提供了不同的模型字典