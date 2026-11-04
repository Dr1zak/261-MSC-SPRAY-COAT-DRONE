from setuptools import setup
setup(name='construction_autonomy',version='0.1.0',packages=['construction_autonomy'],
      data_files=[('share/ament_index/resource_index/packages',['resource/construction_autonomy']),
                  ('share/construction_autonomy',['package.xml'])],
      entry_points={'console_scripts':['construction-test-run=construction_autonomy.cli:main']})
