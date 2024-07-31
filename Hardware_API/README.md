# Pheonix API and Hardware
In previous years, we used the Pheonix v5 API commonly found in First Robotics Systems. They recently in the past 2 years updated to `Pheonix 6 API`. That is what we now will use primarily. These API's are built for the motor controllers that we use.

Our programs using this API ran on RoboRIO systems. We are currently switching away to Canivores and/or Canable. The Canable is a open source version of the Carnivore. 

No matter which CAN device we use of the 3, they all interface with our TalonFX's and Talon SRX's.

## Motors/Motor controllers
We plan to use many different motors now and in the future. The main 2 motors we plan on using this year are: Kraken X60's and Falcon 500's. These are both brushless motors powered by Talon FX systems. The Kraken X60 is newer and more powerful than the Falcon 500. We also will use _linear actuators_, currently we have Progressive Automations linear actuators. These are controlled by Talon SRX controllers so brand does not matter. 

As mentioned above, we have 2 ways of controlling motors: Talon FX and Talon SRX systems. Each controller has benefits as well as negatives. 

- Talon FX: Factory installed on Falcon 500 and Kraken X60 motors. Our intention is to use both of these motors. They can run both Pheonix v5 and v6 versions. The major downside to these controllers, we cant remove and install on different motors.
- Talon SRX: Unlike the Talon FX, these do not come pre-installed on motors. Instead we can wire any motor, including linear actuators, to these controllers. Using Talon SRX's we can use a greater variety of motors on the same CANbus. The downside to the SRX, it is no longer supported and does not interface with the Pheonix 6 API.

## Demos, Examples, Challenges
All demos, examples, or challenges should be completed using the `WPILib VS Code`. All projects can be created, compiled, and used within this version of VS Code. 

## References (not in order/complete)
- Talon SRX: https://store.ctr-electronics.com/talon-srx/
- Talon FX: https://pro.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/index.html
- Falcon 500: https://www.vexrobotics.com/pro/falcon-500
- Kraken X60: https://wcproducts.com/products/kraken
- Pheonix 5: https://v5.docs.ctr-electronics.com/en/stable/ch03_PrimerPhoenixSoft.html
- Pheonix 6: https://pro.docs.ctr-electronics.com/en/latest/
- Progressive Automations: https://www.progressiveautomations.com/
- Canivore: https://store.ctr-electronics.com/canivore/
- Canable: https://makerbase3d.com/product/makerbase-canable-v2/ or https://canable.io/