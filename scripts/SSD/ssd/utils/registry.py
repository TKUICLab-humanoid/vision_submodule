
# 將模型按照模型名稱儲存在指定的模型字典中
def _register_generic(module_dict, module_name, module):
    assert module_name not in module_dict #當 module_name not in module_dict時， 觸發AssertionError
    module_dict[module_name] = module

#這是一個擁有裝飾器函數的類，用途是作為保存模型的字典
class Registry(dict): #繼承 dict:代表將self轉成dict的型態
    """
    A helper class for managing registering modules, it extends a dictionary
    and provides a register functions.
    Eg. creating a registry:
        some_registry = Registry({"default": default_module})
    There're two ways of registering new modules:
    1): normal way is just calling register function:
        def foo():
            ...
        some_registry.register("foo_module", foo)
    2): used as decorator when declaring the module:
        @some_registry.register("foo_module")
        @some_registry.register("foo_module_nickname")
        def foo():
            ...
    Access of module is just like using a dictionary, eg:
        f = some_registry["foo_module"]
    """

    def __init__(self, *args, **kwargs):    
        super(Registry, self).__init__(*args, **kwargs) 

    # register即註冊，意思是將某個模型按照指定的名稱進行存儲
    def register(self, module_name, module=None):
        # 如果存在指定的模型則直接保存該模型
        if module is not None:
            _register_generic(self, module_name, module)
            return

        # 如果不存在模型則本類作為一個裝飾器類，用於保存在別處指定或生成的模型
        def register_fn(fn):
            _register_generic(self, module_name, fn)
            return fn

        return register_fn
