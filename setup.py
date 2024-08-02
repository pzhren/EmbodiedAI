from setuptools import setup,find_packages

with open("README.md", "r") as fh:
    long_description = fh.read()

with open("requirements.txt","rb") as fr:
    requirements = fr.read()

setup(
    name="simulator",
    version="0.0.1",
    author="EmbodiedAI-Lab",
    author_email="",
    description="EmbodiedAI Simulator",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="",
    packages=find_packages(),
    classifiers=[
        "Programming Language ::Python :: 3",
    ],
    python_requires='>=3.10',
)