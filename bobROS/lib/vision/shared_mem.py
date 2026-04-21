import numpy as np
from multiprocessing import shared_memory
import time

class SharedImageWriter:
    def __init__(self, name, shape, dtype=np.uint8):
        self.name = name
        self.shape = shape
        self.dtype = dtype
        self.size = int(np.prod(shape) * np.dtype(dtype).itemsize)
        
        try:
            # Try to attach to existing memory block
            self.shm = shared_memory.SharedMemory(name=self.name)
        except FileNotFoundError:
            # Create if it doesn't exist
            self.shm = shared_memory.SharedMemory(name=self.name, create=True, size=self.size)
        
        self.array = np.ndarray(shape=self.shape, dtype=self.dtype, buffer=self.shm.buf)

    def write(self, frame):
        if frame.shape == self.shape and frame.dtype == self.dtype:
            np.copyto(self.array, frame)
        else:
            print(f"[{self.name}] Shape/type mismatch. Expected {self.shape}{self.dtype}, got {frame.shape}{frame.dtype}")

    def cleanup(self):
        self.shm.close()
        try:
            self.shm.unlink()
        except:
            pass

class SharedImageReader:
    def __init__(self, name, shape, dtype=np.uint8):
        self.name = name
        self.shape = shape
        self.dtype = dtype
        self.shm = None
        self.array = None

    def read(self):
        if self.shm is None:
            try:
                self.shm = shared_memory.SharedMemory(name=self.name)
                self.array = np.ndarray(shape=self.shape, dtype=self.dtype, buffer=self.shm.buf)
            except FileNotFoundError:
                return None
        
        return self.array.copy()

    def cleanup(self):
        if self.shm:
            self.shm.close()
