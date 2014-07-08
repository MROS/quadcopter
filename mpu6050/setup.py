#!/usr/bin/env python2
from distutils.core import setup, Extension

module = Extension('mpu6050',
                   sources = ['sensor_motion.cpp', 'I2Cdev.cpp', 'MPU6050.cpp'],
                   libraries = ['gtkmm-3.0'],
                   define_macros = [('DMP_FIFO_RATE', '9')])

setup (name = 'mpu6050',
        version = '0.1',
        description = 'a Python implementation for MPU6050 chip',
        ext_modules = [module])
