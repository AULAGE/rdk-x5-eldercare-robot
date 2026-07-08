from setuptools import setup

setup(
    name='stm32_to_chassis',
    version='1.0.0',
    packages=[''],
    py_modules=['stm32_to_chassis_bridge'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@example.com',
    description='STM32 chassis protocol to ROS robot chassis bridge',
    license='MIT',
    entry_points={
        'console_scripts': [
            'stm32_to_chassis_bridge = stm32_to_chassis_bridge:main',
        ],
    },
)