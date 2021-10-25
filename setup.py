#! /usr/bin/python3
import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name='pycapacity',
    version='1.0.5',
    author='Antun Skuric',
    author_email='antun.skuric@inria.fr',
    description='A Real-time capable robot capacity calculation module',
    long_description=long_description, #'A Real-time capable robot capacity calculation module', 
    long_description_content_type="text/markdown",
    url='https://gitlab.inria.fr/askuric/pycapacity',
    license='MIT',
    #package_dir = {'': 'src'},
    #packages=setuptools.find_packages(include=['src.*']),
    py_modules=['pycapacity'],
    install_requires=['numpy','scipy']
)