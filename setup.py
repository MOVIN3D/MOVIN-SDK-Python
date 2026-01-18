from setuptools import setup, find_packages

# This setup.py is provided for editable install compatibility
# All configuration is in pyproject.toml
setup(
    name='movin_sdk_python',
    version='0.1.0',
    packages=find_packages(include=['movin_sdk_python', 'movin_sdk_python.*']),
    package_data={
        'movin_sdk_python': ['**/*.xml', '**/*.json', '**/*.STL'],
    },
)

