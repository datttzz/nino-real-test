from glob import glob
from setuptools import find_packages, setup

setup(name='nino_real_rl', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/nino_real_rl']),
                  ('share/nino_real_rl', ['package.xml']),
                  ('share/nino_real_rl/launch', glob('launch/*.py')),
                  ('share/nino_real_rl/config', glob('config/*.yaml'))],
      install_requires=['setuptools', 'numpy', 'PyYAML'], zip_safe=True,
      maintainer='Nino robot owner', maintainer_email='owner@example.com',
      description='Real robot PI and checkpoint SAC adapter; no simulator.',
      license='Apache-2.0',
      entry_points={'console_scripts': [
          'real_controller = nino_real_rl.node:main',
          'import_checkpoint = nino_real_rl.policy:main']})
