import setuptools

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name='pycapacity',
    version='1.0.0',
    author='Antun Skuric',
    author_email='antun.skuric@inria.fr',
    description='A Real-time capable robot capacity calculation module',
    long_description=long_description,
    long_description_content_type="text/markdown",
    url='https://gitlab.inria.fr/askuric/pycapacity',
    license='MIT',
    packages=['pycapacity'],
    install_requires=['numpy','scipy','itertools'],
)