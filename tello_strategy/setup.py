from setuptools import setup

package_name = 'tello_strategy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jamy',
    maintainer_email='jamychahal@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_random = tello_strategy.random_strategy.random_strategy:main',
            'run_void = tello_strategy.void_strategy.void_strategy:main',
            'run_a_cmommt = tello_strategy.A_CMOMMT.A_CMOMMT:main',
            'run_malos = tello_strategy.MALOS.malos_strategy:main',
            'run_i_cmommt = tello_strategy.I_CMOMMT.I_CMOMMT:main',
            'run_land = tello_strategy.LAND.land_strategy:main',
            'run_hi = tello_strategy.HI.HI:main',
            'run_ci = tello_strategy.CI.CI:main',
            'run_ffrl = tello_strategy.FFRL.ffrl_strategy:main',
            'run_f2marl = tello_strategy.F2MARL.f2marl_strategy:main',
            'run_exit = tello_strategy.exit_strategy.exit_strategy:main'
        ],
    },
)
