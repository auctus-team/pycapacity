#! /usr/bin/python3
import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name='pycapacity',
    version='1.2.6',
    author='Antun Skuric',
    author_email='antun.skuric@inria.fr',
    description='A real-time task space capacity calculation module for robotic manipulators and human musculoskeletal models',
    long_description=long_description, #'A Real-time capable robot capacity calculation module', 
    long_description_content_type="text/markdown",
    url='https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity',
    license='MIT',
    #package_dir = {'pycapacity': 'pycapacity'}, 
    packages = ['pycapacity'],
    #py_modules=['pycapacity.robot','pycapacity.human'],
    install_requires=['numpy','scipy','cvxopt>=1.2.6','matplotlib'],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Framework :: Robot Framework",
        "Framework :: Robot Framework :: Library",
        "Framework :: Robot Framework :: Tool"
    ],
)